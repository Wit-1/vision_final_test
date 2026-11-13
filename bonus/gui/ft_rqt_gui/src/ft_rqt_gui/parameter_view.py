"""Mode-aware official parameter editors and a shared horizon for each side."""

import math
import threading

from python_qt_binding.QtCore import Qt, Signal, QTimer
from python_qt_binding.QtWidgets import (QComboBox, QDoubleSpinBox, QHBoxLayout, QLabel,
                                       QPushButton, QScrollArea, QVBoxLayout, QWidget)
from rqt_reconfigure.param_client_widget import ParamClientWidget
from rqt_reconfigure.param_editors import EditorWidget
from rqt_reconfigure.param_groups import GroupWidget

from .processes import master_call
from .tuning import HorizonError, apply_horizon, connect_endpoint


class ParameterView(QWidget):
    discovered = Signal(object, int)
    connected = Signal(object, int, int)
    failed = Signal(str, int, int)
    horizon_done = Signal(object, int)
    warning = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.server = QComboBox()
        self.server.setObjectName('parameter_side')
        self.module = QComboBox()
        self.module.setObjectName('parameter_module')
        self.horizon = QDoubleSpinBox()
        self.horizon.setObjectName('prediction_horizon')
        self.horizon.setDecimals(6)
        self.horizon.setRange(0.0, 5.0)
        self.horizon.setSingleStep(0.01)
        self.horizon.setSuffix(' s')
        self.horizon.setKeyboardTracking(False)
        self.reset_horizon = QPushButton('Reset T')
        self.horizon_note = QLabel()
        self.horizon_note.setWordWrap(True)
        self.note = QLabel()
        self.note.setWordWrap(True)
        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        layout = QVBoxLayout(self)
        layout.addWidget(self.server)
        layout.addWidget(self.module)
        row = QHBoxLayout()
        row.addWidget(QLabel('Prediction horizon T'))
        row.addWidget(self.horizon)
        row.addWidget(self.reset_horizon)
        row.addStretch()
        layout.addLayout(row)
        layout.addWidget(self.horizon_note)
        layout.addWidget(self.note)
        layout.addWidget(self.scroll, 1)
        self._editor = None
        self._session = None
        self._generation = self._editor_generation = 0
        self._closed = self._ready = self._horizon_busy = False
        self._endpoints = {}
        self._errors = {}
        self._blocked_sides = set()
        self._cancelled = threading.Event()
        self._lock = threading.Lock()
        self.server.currentIndexChanged.connect(self._select_side)
        self.module.currentIndexChanged.connect(self._select_module)
        self.horizon.editingFinished.connect(self._apply_horizon)
        self.reset_horizon.clicked.connect(self._reset_horizon)
        self.discovered.connect(self._attach_discovery, Qt.QueuedConnection)
        self.connected.connect(self._attach, Qt.QueuedConnection)
        self.failed.connect(self._failed, Qt.QueuedConnection)
        self.horizon_done.connect(self._horizon_finished, Qt.QueuedConnection)
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh_horizon)
        self._timer.start(300)
        self.set_ready(False)

    def set_session(self, session):
        self.set_ready(False)
        self._session = session
        self._blocked_sides.clear()
        self.server.blockSignals(True)
        self.module.blockSignals(True)
        self.server.clear()
        self.module.clear()
        if session:
            for side in session.sides:
                self.server.addItem('{} ({})'.format(side.label, side.mode), side)
            self.server.setCurrentIndex(next(i for i, side in enumerate(session.sides) if side.primary))
            self.module.addItem('Detection', 'ft_detector')
            if session.options.tracker:
                self.module.addItem('Linear tracking', 'ft_tracker')
            if session.options.selector:
                self.module.addItem('Rotating tracking', 'ft_selector')
        self.module.blockSignals(False)
        self.server.blockSignals(False)

    def set_ready(self, ready):
        self._ready = ready
        self.server.setEnabled(ready)
        self.module.setEnabled(ready)
        if not ready:
            self.clear()

    def _select_side(self, *_):
        if self._ready:
            self.connect_server()

    def _select_module(self, *_):
        if self._ready:
            self._open_editor()

    def _current(self, generation):
        return not self._closed and generation == self._generation

    def connect_server(self):
        self.set_ready(True)
        self.clear()
        side = self.server.currentData()
        if side is None:
            return
        self.note.setText('Checking parameter services for ' + side.label + '...')
        generation = self._generation
        modules = [self.module.itemData(i) for i in range(self.module.count())]

        def discover():
            endpoints, errors = {}, {}
            try:
                services = {name for name, _ in master_call('getSystemState')[2]}
                for module in modules:
                    if not self._current(generation):
                        break
                    server = side.namespace + '/' + module + '/tuning'
                    if server + '/set_parameters' not in services:
                        errors[module] = 'This workspace does not provide online tuning for this module.'
                        continue
                    try:
                        endpoints[module] = connect_endpoint(server)
                    except Exception as error:
                        errors[module] = 'Parameter connection failed: ' + str(error)
            except Exception as error:
                errors = {module: 'Parameter discovery failed: ' + str(error) for module in modules}
            with self._lock:
                if self._current(generation):
                    self.discovered.emit((endpoints, errors), generation)
                else:
                    for endpoint in endpoints.values():
                        endpoint.client.close()

        threading.Thread(target=discover, daemon=True).start()

    def _attach_discovery(self, result, generation):
        endpoints, errors = result
        if not self._current(generation):
            for endpoint in endpoints.values():
                endpoint.client.close()
            return
        self._endpoints, self._errors = endpoints, errors
        self._refresh_horizon()
        self._open_editor()

    def _open_editor(self):
        self._close_editor()
        module = self.module.currentData()
        endpoint = self._endpoints.get(module)
        if not endpoint:
            self.note.setText(self._errors.get(module, 'Checking parameter services...'))
            return
        generation, editor_generation = self._generation, self._editor_generation
        server = endpoint.client.name
        self.note.setText('Connecting to ' + server + '...')

        def connect():
            try:
                result = connect_endpoint(server)
                with self._lock:
                    if self._current(generation) and editor_generation == self._editor_generation:
                        self.connected.emit(result, generation, editor_generation)
                    else:
                        result.client.close()
            except Exception as error:
                if self._current(generation):
                    self.failed.emit(str(error), generation, editor_generation)

        threading.Thread(target=connect, daemon=True).start()

    def _attach(self, endpoint, generation, editor_generation):
        if not self._current(generation) or editor_generation != self._editor_generation:
            endpoint.client.close()
            return
        side = self.server.currentData()
        module = self.module.currentData()
        self._editor = ParamClientWidget(endpoint.client, endpoint.client.name)
        self._filter_fields(self._editor, module, side.mode)
        self._editor.setEnabled(not self._horizon_busy)
        self.scroll.setWidget(self._editor)
        self.note.setText(endpoint.client.name + ' · Edits affect only this side. Restart reloads its YAML values.')

    @staticmethod
    def _filter_fields(group, module, mode):
        # 同时隐藏输入控件和标签，空组也隐藏；不改服务端参数描述与协议。
        any_visible = False
        for widget in group.editor_widgets:
            if isinstance(widget, GroupWidget):
                visible = ParameterView._filter_fields(widget, module, mode)
            elif isinstance(widget, EditorWidget):
                name = widget.param_name
                if module == 'ft_detector':
                    visible = (name.startswith('nn_') or name == 'large_armor_min_spacing_ratio')
                    if mode != 'nn':
                        visible = not name.startswith('nn_')
                else:
                    visible = name != 'prediction_time_s'
                label = group.grid.labelForField(widget)
                if label:
                    label.setVisible(visible)
            else:
                continue
            widget.setVisible(visible)
            any_visible = any_visible or visible
        return any_visible

    def _failed(self, message, generation, editor_generation):
        if self._current(generation) and editor_generation == self._editor_generation:
            self.note.setText('Parameter connection failed. Select the module again to retry.')
            self.warning.emit('Parameter connection failed: ' + message)

    def _prediction_endpoints(self):
        modules = [self.module.itemData(i) for i in range(self.module.count())
                   if self.module.itemData(i) != 'ft_detector']
        endpoints = [self._endpoints.get(module) for module in modules]
        if not endpoints or any(endpoint is None or 'prediction_time_s' not in endpoint.fields
                                for endpoint in endpoints):
            return []
        return endpoints

    def _refresh_horizon(self):
        if self._horizon_busy:
            return
        endpoints = self._prediction_endpoints() if self._ready else []
        blocked = self.server.currentIndex() in self._blocked_sides
        enabled = bool(endpoints) and not blocked
        self.horizon.setEnabled(enabled)
        self.reset_horizon.setEnabled(False)
        if not enabled:
            if blocked:
                self.horizon_note.setText('Horizon consistency is unconfirmed. Apply / Restart is required.')
            elif self._ready:
                self.horizon_note.setText('Shared T requires online horizon support in every enabled prediction module.')
            else:
                self.horizon_note.setText('Start a session to adjust prediction horizon T.')
            return
        fields = [endpoint.fields['prediction_time_s'] for endpoint in endpoints]
        lower = max(field['min'] for field in fields)
        upper = min(field['max'] for field in fields)
        if lower > upper:
            self.horizon.setEnabled(False)
            self.horizon_note.setText('The enabled prediction modules have no common T range.')
            return
        self.horizon.setRange(lower, upper)
        values = [(endpoint.client.config or {}).get('prediction_time_s') for endpoint in endpoints]
        defaults = [field['default'] for field in fields]
        same_default = all(math.isclose(defaults[0], value, abs_tol=1e-9) for value in defaults)
        self.reset_horizon.setEnabled(same_default)
        self.reset_horizon.setToolTip('Restore the startup YAML horizon.' if same_default else
                                     'The enabled modules have different startup horizons.')
        if any(value is None for value in values):
            self.horizon_note.setText('Waiting for prediction parameter values...')
        elif not all(math.isclose(values[0], value, abs_tol=1e-9) for value in values):
            self.horizon_note.setText('Prediction horizons differ. Enter one T to apply to all enabled modules on this side.')
        else:
            if not self.horizon.hasFocus():
                self.horizon.setValue(values[0])
            self.horizon_note.setText('T applies to every enabled prediction module on this side. New frames use the updated value.')

    def _reset_horizon(self):
        endpoints = self._prediction_endpoints()
        if endpoints:
            self.horizon.setValue(endpoints[0].fields['prediction_time_s']['default'])
            self._apply_horizon()

    def _apply_horizon(self):
        if self._horizon_busy or not self.horizon.isEnabled():
            return
        endpoints = self._prediction_endpoints()
        value = self.horizon.value()
        if not endpoints:
            return
        if all(math.isclose((endpoint.client.config or {}).get('prediction_time_s', -1), value, abs_tol=1e-9)
               for endpoint in endpoints):
            return
        generation, cancelled = self._generation, self._cancelled
        self._horizon_busy = True
        for widget in (self.server, self.module, self.horizon, self.reset_horizon, self._editor):
            if widget is not None:
                widget.setEnabled(False)
        self.horizon_note.setText('Applying T to this side...')

        def update():
            error = None
            try:
                apply_horizon(endpoints, value, cancelled)
            except HorizonError as failure:
                error = failure
            if self._current(generation):
                self.horizon_done.emit(error, generation)

        threading.Thread(target=update, daemon=True).start()

    def _horizon_finished(self, error, generation):
        if not self._current(generation):
            return
        self._horizon_busy = False
        self.server.setEnabled(self._ready)
        self.module.setEnabled(self._ready)
        if self._editor:
            self._editor.setEnabled(True)
        if error:
            if error.inconsistent:
                self._blocked_sides.add(self.server.currentIndex())
            self.warning.emit(str(error))
        self._refresh_horizon()
        if error:
            self.horizon_note.setText(str(error))

    def _close_editor(self):
        self._editor_generation += 1
        if self._editor:
            editor = self.scroll.takeWidget()
            self._editor = None
            editor.reconf.config_callback = None
            editor.close()

    def clear(self):
        with self._lock:
            self._generation += 1
            self._cancelled.set()
            self._cancelled = threading.Event()
        self._horizon_busy = False
        self._close_editor()
        for endpoint in self._endpoints.values():
            endpoint.client.close()
        self._endpoints, self._errors = {}, {}
        self.horizon.setEnabled(False)
        self.reset_horizon.setEnabled(False)
        self.note.setText('Start a session to connect. Edits are not saved to YAML.')
        self.horizon_note.setText('Start a session to adjust prediction horizon T.')

    def shutdown(self):
        if self._closed:
            return
        self._closed = True
        self._timer.stop()
        self.clear()
