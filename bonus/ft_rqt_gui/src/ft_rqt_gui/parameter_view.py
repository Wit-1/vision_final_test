"""Reuse rqt_reconfigure's editor and select which side receives parameter edits."""

import threading

from dynamic_reconfigure.client import Client
from python_qt_binding.QtCore import Qt, Signal
from python_qt_binding.QtWidgets import QComboBox, QLabel, QScrollArea, QVBoxLayout, QWidget
from rqt_reconfigure.param_client_widget import ParamClientWidget


class ParameterView(QWidget):
    connected = Signal(object, int)
    failed = Signal(str, int)
    warning = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.server = QComboBox()
        self.server.setObjectName('parameter_side')
        self.note = QLabel('Start a session to connect to the selected detector. Edits are not saved to YAML.')
        self.note.setWordWrap(True)
        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        layout = QVBoxLayout(self)
        layout.addWidget(self.server)
        layout.addWidget(self.note)
        layout.addWidget(self.scroll, 1)
        self._editor = None
        self._generation = 0
        self._closed = self._ready = False
        self._lock = threading.Lock()
        self.server.setEnabled(False)
        self.server.currentIndexChanged.connect(self._select)
        self.connected.connect(self._attach, Qt.QueuedConnection)
        self.failed.connect(self._failed, Qt.QueuedConnection)

    def set_session(self, session):
        self.set_ready(False)
        self.server.blockSignals(True)
        self.server.clear()
        if session:
            for side in session.sides:
                self.server.addItem(side.label + ' — ' + side.tuning, side.tuning)
            self.server.setCurrentIndex(next(i for i, side in enumerate(session.sides) if side.primary))
        self.server.blockSignals(False)

    def set_ready(self, ready):
        self._ready = ready
        self.server.setEnabled(ready)
        if not ready:
            self.clear()

    def _select(self, *_):
        if self._ready:
            self.connect_server()

    def connect_server(self):
        self.set_ready(True)
        self.clear()
        generation = self._generation
        server = self.server.currentData()
        if not server:
            return
        self.note.setText('Connecting to ' + server + '...')

        def connect():
            # Client 后台连接；代次检查使切换左右节点后的旧回调失效。
            client = None
            try:
                client = Client(server, timeout=3.0)
                client.get_configuration(timeout=3.0)
                with self._lock:
                    if self._closed or generation != self._generation:
                        client.close()
                        return
                    self.connected.emit(client, generation)
            except Exception as error:
                if client:
                    client.close()
                with self._lock:
                    if not self._closed and generation == self._generation:
                        self.failed.emit(str(error), generation)

        threading.Thread(target=connect, daemon=True).start()

    def _attach(self, client, generation):
        if self._closed or generation != self._generation:
            client.close()
            return
        server = self.server.currentData()
        self._editor = ParamClientWidget(client, server)
        self.scroll.setWidget(self._editor)
        self.note.setText(server + ' · Edits affect only this side. Restart reloads its YAML values.')

    def _failed(self, message, generation):
        if not self._closed and generation == self._generation:
            self.note.setText('Parameter connection failed. Select a side again or restart to retry.')
            self.warning.emit('Parameter connection failed: ' + message)

    def clear(self):
        with self._lock:
            self._generation += 1
        if self._editor:
            editor = self.scroll.takeWidget()
            self._editor = None
            editor.reconf.config_callback = None
            editor.close()
        self.note.setText('Start a session to connect to the selected detector. Edits are not saved to YAML.')

    def shutdown(self):
        if self._closed:
            return
        self._closed = True
        self.clear()
