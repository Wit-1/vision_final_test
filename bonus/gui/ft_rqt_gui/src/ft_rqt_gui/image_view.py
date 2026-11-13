"""Display the newest ROS image in Qt without accumulating old frames."""

import threading

import cv2
from cv_bridge import CvBridge, CvBridgeError
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtGui import QImage, QPixmap
from python_qt_binding.QtWidgets import QComboBox, QLabel, QSizePolicy, QVBoxLayout, QWidget
import rospy
from sensor_msgs.msg import CompressedImage, Image


class _Canvas(QLabel):
    def __init__(self):
        super().__init__('Start a session to view images.')
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(320, 240)
        self.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self.setStyleSheet('background: #20252b; color: #d7dce2; border-radius: 4px;')
        self.original = None

    def show_image(self, image):
        self.original = QPixmap.fromImage(image)
        self._fit()

    def _fit(self):
        if self.original is not None:
            self.setPixmap(self.original.scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._fit()

    def reset(self):
        self.original = None
        self.clear()
        self.setText('Waiting for session images...')


class ImageView(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.source = QComboBox()
        self.source.setObjectName('image_source')
        self.source.addItem('Original image', 'raw')
        self.source.addItem('Primary detection result', 'debug')
        self.source.setCurrentIndex(1)
        self.canvas = _Canvas()
        self.info = QLabel('Original image timestamps; display refresh is limited to 15 Hz.')
        self.info.setWordWrap(True)
        layout = QVBoxLayout(self)
        layout.addWidget(self.source)
        layout.addWidget(self.canvas, 1)
        layout.addWidget(self.info)
        self.bridge = CvBridge()
        self._closed = False
        self._lock = threading.Lock()
        self._latest = {}
        self._subscribers = []
        self._rendered = None
        self._topics = {}
        self._generation = 0
        self._session = None
        self.source.currentIndexChanged.connect(self._select_source)
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._render)
        self._timer.start(67)

    def set_session(self, session):
        self._session = session
        self.source.blockSignals(True)
        self.source.clear()
        self.source.addItem('Original image', 'raw')
        if session:
            for index, side in enumerate(session.sides):
                self.source.addItem(side.label + ' detection (' + side.mode + ')', 'side' + str(index))
                if session.options.selector:
                    self.source.addItem(side.label + ' prediction overlay', 'prediction' + str(index))
            if session.options.compare:
                self.source.addItem('Comparison: ' + session.sides[0].label + ' / ' + session.sides[1].label, 'comparison')
                if session.options.selector:
                    self.source.addItem('Prediction comparison: ' + session.sides[0].label + ' / ' + session.sides[1].label,
                                        'prediction_comparison')
        self.source.setCurrentIndex(self.source.count() - 1)
        self.source.blockSignals(False)
        # 各图源只缓存最近一帧，暂停后也能直接切换原图、检测图和对比图。
        with self._lock:
            self._generation += 1
            generation = self._generation
            self._latest = {}
        for subscriber in self._subscribers:
            subscriber.unregister()
        self._subscribers = []
        self._topics = {}
        if session:
            self._topics = {'raw': session.image_topic}
            self._topics.update({'side'+str(i): side.prefix + '/debug/image' for i, side in enumerate(session.sides)})
            if session.options.selector:
                self._topics.update({'prediction'+str(i): side.prefix + '/debug/prediction_image'
                                     for i, side in enumerate(session.sides)})
            if session.options.compare:
                self._topics['comparison'] = '/ft/debug/comparison'
                if session.options.selector:
                    self._topics['prediction_comparison'] = '/ft/debug/prediction_comparison_image'
            for source, topic in self._topics.items():
                def receive(message, key=source):
                    # ROS 线程只保存最近消息；代次检查防止旧回调污染新会话。
                    with self._lock:
                        if generation == self._generation:
                            self._latest[key] = message
                self._subscribers.append(rospy.Subscriber(
                    topic, CompressedImage if source == 'raw' else Image,
                    receive, queue_size=1, buff_size=16 * 1024 * 1024))
        self._select_source()

    def _select_source(self, *_):
        self._rendered = None
        self.canvas.reset()
        topic = self._topics.get(self.source.currentData())
        self.info.setText('Waiting for topic: ' + topic if topic else 'No active session.')
        self._render()

    def _render(self):
        source = self.source.currentData()
        with self._lock:
            message = self._latest.get(source)
        if message is None or message is self._rendered:
            return
        try:
            # RGB 转换直接由 cv_bridge 完成；copy 使 QImage 不再借用临时数组。
            rgb = (self.bridge.compressed_imgmsg_to_cv2(message, 'rgb8') if source == 'raw' else
                   self.bridge.imgmsg_to_cv2(message, 'rgb8'))
            if rgb is None or rgb.ndim != 3 or rgb.shape[2] != 3:
                raise ValueError('The image did not decode to three-channel RGB.')
            height, width = rgb.shape[:2]
            image = QImage(rgb.data, width, height, rgb.strides[0], QImage.Format_RGB888).copy()
            self.canvas.show_image(image)
            self._rendered = message
            self.info.setText('{} × {}  |  {}  |  {:.6f} s'.format(
                width, height, message.header.frame_id, message.header.stamp.to_sec()))
        except (CvBridgeError, cv2.error, ValueError, TypeError) as error:
            self.info.setText('Image display failed: ' + str(error))

    def shutdown(self):
        if self._closed:
            return
        self._closed = True
        self._timer.stop()
        self.set_session(None)
