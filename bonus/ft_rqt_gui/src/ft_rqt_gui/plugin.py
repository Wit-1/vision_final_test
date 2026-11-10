"""One rqt dock containing session controls, an image viewer and parameter editor."""

import math
import re

from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QFormLayout, QGroupBox, QHBoxLayout,
    QLabel, QLineEdit, QFileDialog, QPlainTextEdit, QPushButton, QScrollArea, QSplitter, QTabWidget, QVBoxLayout, QWidget,
)
from rqt_gui_py.plugin import Plugin

from .configuration import Catalog, Options, project_root
from .image_view import ImageView
from .parameter_view import ParameterView
from .session_controller import SessionController


class ControlPanel(Plugin):
    def __init__(self, context):
        super().__init__(context)
        self.setObjectName('VisionControlPanel')
        self._closed = False
        self.catalog = Catalog(project_root())
        self._session = None
        self._parameters_connected = False
        self._state = 'idle'
        self._widget = QWidget()
        self._widget.setObjectName('vision_control_panel')
        self._widget.setWindowTitle('Armor Debug Console')
        self._widget.setMinimumSize(1080, 720)
        self._build_ui()
        context.add_widget(self._widget)
        self.controller = SessionController(self.catalog, self._widget)
        self.controller.state_changed.connect(self._on_state, Qt.QueuedConnection)
        self.controller.session_changed.connect(self._on_session, Qt.QueuedConnection)
        self.controller.log.connect(self._log, Qt.QueuedConnection)
        self.parameters.warning.connect(self._log)
        self.start_button.clicked.connect(self._start)
        self.apply_button.clicked.connect(self._start)
        self.stop_button.clicked.connect(self.controller.stop)
        self.pause_button.clicked.connect(self.controller.pause)
        for widget in (self.dataset, self.mode, self.compare):
            widget.currentIndexChanged.connect(self._pending)
        self.rate.currentTextChanged.connect(self._pending)
        self.start_time.valueChanged.connect(self._pending)
        for widget in (self.tracker, self.selector, self.loop, self.rviz, self.compare_rviz):
            widget.toggled.connect(self._pending)
        self.compare.currentIndexChanged.connect(self._comparison_changed)
        self.compare_rviz.toggled.connect(lambda checked: self.rviz.setChecked(True) if checked else None)
        self.rviz.toggled.connect(lambda checked: self.compare_rviz.setChecked(False) if not checked else None)
        self.baseline.textChanged.connect(self._pending)
        self.candidate.textChanged.connect(self._pending)
        self._comparison_changed()
        self._on_state('idle', 'Select a configuration, then click Start.')
        self._log('This console manages only its own session. Use Apply / Restart after changing settings.')
        QTimer.singleShot(0, lambda: self._widget.window().resize(1440, 920))

    @staticmethod
    def _group(title, parent_layout):
        group = QGroupBox(title)
        layout = QFormLayout(group)
        layout.setSpacing(9)
        parent_layout.addWidget(group)
        return layout

    def _build_ui(self):
        """界面控件只表达运行选项，算法参数继续由现有 YAML 和调参服务管理。"""
        layout = QVBoxLayout(self._widget)
        layout.setContentsMargins(14, 12, 14, 12)
        header = QHBoxLayout()
        title = QLabel('Armor Debug Console')
        title.setStyleSheet('font-size: 21px; font-weight: bold;')
        header.addWidget(title)
        header.addStretch()
        self.status_badge = QLabel('Idle')
        self.status_badge.setMinimumWidth(90)
        self.status_badge.setAlignment(Qt.AlignCenter)
        header.addWidget(self.status_badge)
        layout.addLayout(header)
        self.session_label = QLabel('Select a dataset, detection mode and playback settings.')
        self.session_label.setWordWrap(True)
        layout.addWidget(self.session_label)

        splitter = QSplitter(Qt.Horizontal)
        left = QWidget()
        left.setMinimumWidth(280)
        left.setMaximumWidth(380)
        controls = QVBoxLayout(left)
        controls.setContentsMargins(0, 0, 12, 0)
        form = self._group('Dataset and detection', controls)
        self.dataset = QComboBox()
        self.dataset.setObjectName('dataset')
        for name in self.catalog.profiles:
            self.dataset.addItem(name, name)
        self.dataset.setCurrentIndex(max(0, self.dataset.findData('blue_1')))
        self.mode = QComboBox()
        self.mode.setObjectName('mode')
        self.mode.addItem('Traditional', 'tradition')
        self.mode.addItem('Neural network (NN)', 'nn')
        self.compare = QComboBox()
        for label, value in (('Off', 'off'), ('Detection modes', 'modes'), ('Workspaces', 'workspaces')):
            self.compare.addItem(label, value)
        self.compare.setObjectName('compare')
        form.addRow('Dataset', self.dataset)
        form.addRow('Primary mode', self.mode)
        form.addRow('Comparison', self.compare)

        form = self._group('Workspaces (already built)', controls)
        self.workspace_group = form.parentWidget()
        self.baseline = self._workspace_picker('Baseline', self.catalog.root.parent / 'vision_final_test_baseline', form)
        self.candidate = self._workspace_picker('Candidate', self.catalog.root, form)

        form = self._group('Pipeline', controls)
        self.tracker = QCheckBox('Single-armor tracking')
        self.selector = QCheckBox('Rotation and armor selection')
        self.rviz = QCheckBox('Open RViz')
        self.compare_rviz = QCheckBox('Compare in RViz')
        self.compare_rviz.setObjectName('compare_rviz')
        self.compare_rviz.setToolTip('Open two RViz windows with linked Orbit rotation, pan and zoom.')
        self.rviz.setToolTip('Open RViz in a separate window when the session starts.')
        for name, widget in (('tracker', self.tracker), ('selector', self.selector), ('rviz', self.rviz), ('compare_rviz', self.compare_rviz)):
            widget.setObjectName(name)
            form.addRow(widget)
        self.tracker.setChecked(True)
        self.selector.setChecked(True)

        form = self._group('Playback', controls)
        self.rate = QComboBox()
        self.rate.setObjectName('rate')
        self.rate.setEditable(True)
        self.rate.addItems(['0.1', '0.2', '0.5', '1.0', '2.0'])
        self.rate.setCurrentText('0.2')
        self.start_time = QDoubleSpinBox()
        self.start_time.setObjectName('start_time')
        self.start_time.setRange(0, 864000)
        self.start_time.setDecimals(3)
        self.start_time.setSuffix(' s')
        self.loop = QCheckBox('Loop playback')
        self.loop.setObjectName('loop')
        form.addRow('Speed', self.rate)
        form.addRow('Start time', self.start_time)
        form.addRow(self.loop)

        self.pending = QLabel('No configuration applied.')
        self.pending.setWordWrap(True)
        self.pending.setStyleSheet('color: #956319; padding: 3px;')
        controls.addWidget(self.pending)
        row = QHBoxLayout()
        self.start_button = QPushButton('Start')
        self.start_button.setObjectName('start_session')
        self.apply_button = QPushButton('Apply / Restart')
        self.apply_button.setObjectName('apply_session')
        self.start_button.setStyleSheet(
            'QPushButton {background: #2e6da4; color: white; padding: 7px;}'
            'QPushButton:disabled {background: #aebbc6; color: #f5f5f5;}')
        row.addWidget(self.start_button)
        row.addWidget(self.apply_button)
        controls.addLayout(row)
        row = QHBoxLayout()
        self.pause_button = QPushButton('Pause')
        self.pause_button.setObjectName('pause_playback')
        self.stop_button = QPushButton('Stop')
        self.stop_button.setObjectName('stop_session')
        row.addWidget(self.pause_button)
        row.addWidget(self.stop_button)
        controls.addLayout(row)
        hint = QLabel('Apply / Restart uses the selected start time.\nPause / Resume keeps the pipeline running.')
        hint.setWordWrap(True)
        hint.setStyleSheet('color: #666; padding-top: 8px;')
        controls.addWidget(hint)
        controls.addStretch()
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(left)
        scroll.setMinimumWidth(300)
        scroll.setMaximumWidth(400)
        scroll.setFrameShape(QScrollArea.NoFrame)
        splitter.addWidget(scroll)
        self.tabs = QTabWidget()
        self.images = ImageView()
        self.parameters = ParameterView()
        self.tabs.addTab(self.images, 'Images')
        self.tabs.addTab(self.parameters, 'Parameters')
        splitter.addWidget(self.tabs)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([310, 1050])
        layout.addWidget(splitter, 1)

        self.status = QLabel()
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.logs = QPlainTextEdit()
        self.logs.setObjectName('session_log')
        self.logs.setReadOnly(True)
        self.logs.setMaximumBlockCount(400)
        self.logs.setMaximumHeight(130)
        layout.addWidget(self.logs)
        self.settings_widgets = (self.dataset, self.mode, self.compare, self.tracker, self.selector,
                                 self.rviz, self.compare_rviz, self.baseline, self.candidate, self.rate, self.start_time, self.loop)

    def _options(self):
        rate = float(self.rate.currentText())
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError('Playback speed must be a positive number.')
        return Options(dataset=self.dataset.currentData(), mode=self.mode.currentData(),
                       comparison=self.compare.currentData(), tracker=self.tracker.isChecked(),
                       selector=self.selector.isChecked(), rate=rate, start=self.start_time.value(),
                       loop=self.loop.isChecked(), rviz=self.rviz.isChecked(),
                       compare_rviz=self.compare_rviz.isChecked(), baseline=self.baseline.text(),
                       candidate=self.candidate.text())

    def _workspace_picker(self, label, path, form):
        row = QHBoxLayout()
        edit = QLineEdit(str(path))
        edit.setObjectName(label.lower() + '_workspace')
        edit.setToolTip('Select the workspace root visible in this environment. Build it before starting.')
        button = QPushButton('...')
        button.setMaximumWidth(32)
        button.clicked.connect(lambda: self._browse_workspace(edit))
        row.addWidget(edit)
        row.addWidget(button)
        form.addRow(label, row)
        return edit

    def _browse_workspace(self, edit):
        path = QFileDialog.getExistingDirectory(self._widget, 'Select built workspace', edit.text())
        if path:
            edit.setText(path)

    def _comparison_changed(self, *_):
        self.workspace_group.setVisible(self.compare.currentData() == 'workspaces')
        enabled = self.compare.currentData() != 'off' and self._state not in ('starting', 'stopping', 'pausing')
        self.compare_rviz.setEnabled(enabled)
        if self.compare.currentData() == 'off':
            self.compare_rviz.setChecked(False)

    def _pending(self, *_):
        try:
            options = self._options()
            self.pending.setText('Pending changes: click Apply / Restart.' if self._session and options != self._session.options else
                                 'Configuration applied.' if self._session else 'No configuration applied.')
        except ValueError:
            self.pending.setText('Enter a valid positive playback speed.')

    def _start(self):
        try:
            self.controller.start(self._options())
        except ValueError as error:
            self.status.setText(str(error))
            self._log(str(error))

    def _on_session(self, session):
        if self._closed:
            return
        self._session = session
        self._parameters_connected = False
        self.parameters.set_session(session)
        self.images.set_session(session)
        if session:
            options = session.options
            self.session_label.setText('{}  |  Primary: {}{}  |  Duration: {:.2f} s  |  Start: {:.3f} s'.format(
                options.dataset, options.mode, ' + ' + options.comparison if options.compare else '', session.duration, options.start))
            self.session_label.setToolTip('\n'.join(side.label + ': ' + str(side.workspace.root) for side in session.sides))
        else:
            self.session_label.setText('Select a dataset, detection mode and playback settings.')
        self._pending()

    def _on_state(self, state, message):
        if self._closed:
            return
        self._state = state
        labels = {'idle': 'Idle', 'starting': 'Starting', 'running': 'Playing', 'paused': 'Paused',
                  'pausing': 'Updating', 'stopping': 'Stopping', 'finished': 'Finished', 'error': 'Error'}
        self.status_badge.setText(labels[state])
        color = '#a32929' if state == 'error' else '#286a45' if state == 'running' else '#536473'
        self.status_badge.setStyleSheet('background: {}; color: white; padding: 6px; border-radius: 4px;'.format(color))
        self.status.setText(message)
        busy = state in ('starting', 'stopping', 'pausing')
        for widget in self.settings_widgets:
            widget.setEnabled(not busy)
        self.workspace_group.setEnabled(not busy)
        self._comparison_changed()
        self.start_button.setEnabled(not busy and state in ('idle', 'error'))
        self.apply_button.setEnabled(not busy and self._session is not None)
        self.stop_button.setEnabled(state not in ('idle', 'stopping'))
        self.pause_button.setEnabled(state in ('running', 'paused'))
        self.pause_button.setText('Resume' if state == 'paused' else 'Pause')
        if state in ('starting', 'stopping'):
            self.parameters.set_ready(False)
            self._parameters_connected = False
        if state in ('running', 'paused', 'finished') and self._session and not self._parameters_connected:
            self.parameters.connect_server()
            self._parameters_connected = True
        if state in ('error', 'finished'):
            self._log(message)

    def _log(self, message):
        if self._closed:
            return
        self.logs.appendPlainText(re.sub(r'\x1b\[[0-9;]*m', '', message))

    def shutdown_plugin(self):
        if self._closed:
            return
        self._closed = True
        self.parameters.shutdown()
        self.images.shutdown()
        self.controller.shutdown()
