"""Create per-session RViz configs without changing the project's saved config."""

from copy import deepcopy
from pathlib import Path
import tempfile
import threading
import time
import uuid

import rospy
from sensor_msgs.msg import CameraInfo
import yaml

from .processes import OwnedProcess, node_identity


class Viewers:
    def __init__(self, root, session, log):
        self.root, self.session, self.log = root, session, log
        self.channel = '/ft/gui/view_sync/' + uuid.uuid4().hex
        self.directory = tempfile.TemporaryDirectory(prefix='ft-rviz-')
        self.processes = {}
        self.identities = {}
        self.started = 0.0
        self.frame = ''
        self.active_frame = ''
        self._lock = threading.Lock()
        self.subscriber = rospy.Subscriber(session.camera_info_topic, CameraInfo, self._camera, queue_size=1)

    def _camera(self, message):
        if message.width and message.height and message.header.frame_id:
            with self._lock:
                self.frame = message.header.frame_id

    def _start(self, frame):
        # TF 话题按进程重映射；相同 armor_0 字符串也不会混入另一侧的 TF 缓存。
        template = yaml.safe_load((self.root / 'src/ft_bringup/rviz/vision.rviz').read_text())
        sides = self.session.sides if self.session.options.compare_rviz else (self.session.primary,)
        for index, side in enumerate(sides):
            name = '/ft_gui_rviz_primary' if side.primary else '/ft_gui_rviz_other'
            title = '{} ({})'.format(side.label, side.mode)
            config = deepcopy(template)
            manager = config['Visualization Manager']
            manager['Global Options']['Fixed Frame'] = frame
            for display in manager['Displays']:
                if 'Marker Topic' in display:
                    display['Marker Topic'] = display['Marker Topic'].replace('/ft/', side.prefix + '/', 1)
                if display.get('Name') == 'Prediction comparison':
                    display['Enabled'] = bool(self.session.options.compare and self.session.options.tracker and
                                              self.session.options.selector)
            if self.session.options.compare_rviz:
                config['Panels'].append(dict(Class='ft_rqt_gui/OrbitSync', Name='Linked Orbit view',
                                             Channel=self.channel, Side='left' if index == 0 else 'right', Label=title))
            config['Window Geometry'].update(Width=960, Height=760, X=30 + index * 980, Y=50)
            path = Path(self.directory.name) / (side.label + '_' + side.mode + '.rviz')
            path.write_text(yaml.safe_dump(config))
            command = ['rosrun', 'rviz', 'rviz', '-d', str(path), '-f', frame,
                       '__name:=' + name.lstrip('/'), '/tf:=' + side.tf_topic,
                       '/tf_static:=' + ('/tf_static' if side.primary else side.prefix + '/tf_static')]
            self.processes[name] = OwnedProcess(command, self.root, lambda line, label=title: self.log('[RViz '+label+'] '+line))
            self.log('Opening RViz: {} | {}'.format(title, side.workspace.root))
        self.active_frame = frame
        self.started = time.monotonic()

    def poll(self):
        with self._lock:
            frame = self.frame
        if not frame:
            return
        if frame != self.active_frame:
            self._stop_processes()
            self._start(frame)
        for name, process in self.processes.items():
            process.remember()
            if process.process.poll() is not None:
                raise RuntimeError('An RViz window exited: ' + name)
            if name not in self.identities:
                try:
                    identity = node_identity(name)
                    if identity[1] not in process.alive_pids():
                        raise RuntimeError('RViz node is owned by an external process: ' + name)
                    self.identities[name] = identity
                except (OSError, RuntimeError):
                    if time.monotonic() - self.started > 20:
                        raise RuntimeError('RViz did not become ready: ' + name)

    def _stop_processes(self):
        for name in list(self.processes):
            self.processes[name].stop(grace_period=2.0)
            del self.processes[name]
        self.identities.clear()

    def stop(self):
        self.subscriber.unregister()
        self._stop_processes()
        self.directory.cleanup()
