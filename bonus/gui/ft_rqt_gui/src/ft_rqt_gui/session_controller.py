"""Serialize GUI requests in a worker; ROS time never drives lifecycle checks."""

import queue
import threading
import time

from python_qt_binding.QtCore import QObject, Signal
import rospy
from std_srvs.srv import SetBool

from .processes import OwnedProcess, master_call, node_identity
from .viewers import Viewers
from .workspace import verify_loaded


PLAYER = '/ft_gui_player'
BUSINESS_NODES = {'/ft_detector', '/ft_tracker', '/ft_selector', '/ft_comparison/ft_detector',
                  '/compare_detections', '/compare_predictions', '/ft_rviz_launcher', '/ft_rviz', PLAYER,
                  '/ft_gui_other/ft_detector', '/ft_gui_other/ft_tracker', '/ft_gui_other/ft_selector',
                  '/ft_gui_other/compare_predictions', '/ft_gui_comparison',
                  '/ft_gui_rviz_primary', '/ft_gui_rviz_other'}
BUSINESS_NODES.update({'/prediction_overlay', '/ft_gui_overlay_primary', '/ft_gui_overlay_other',
                       '/ft_gui_prediction_comparison'})


class Cancelled(Exception):
    pass


class SessionController(QObject):
    state_changed = Signal(str, str)
    log = Signal(str)
    session_changed = Signal(object)

    def __init__(self, catalog, parent=None):
        super().__init__(parent)
        self.catalog = catalog
        self.state = 'idle'
        self.active = None
        self.pipelines = []
        self.player = self.viewers = None
        self._business = {}
        self.expected = set()
        self._nodes = {}
        self._queue = queue.Queue()
        self._cancel = threading.Event()
        self._lock = threading.Lock()
        self._busy = False
        self._last_check = 0.0
        self._missing = 0
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _state(self, state, message):
        # 先释放操作锁再通知界面，避免按钮已启用而后台仍拒绝下一次点击。
        if state in ('idle', 'running', 'paused', 'finished', 'error'):
            with self._lock:
                self._busy = not self._queue.empty()
        self.state = state
        self.state_changed.emit(state, message)

    def start(self, options):
        """请求立即占用操作锁，防止 GUI 双击排入两次启动。"""
        with self._lock:
            if self._busy:
                return False
            self._busy = True
            self._cancel.clear()
        self._state('starting', 'Checking configuration and starting...')
        self._queue.put(('start', options))
        return True

    def stop(self):
        self._cancel.set()
        with self._lock:
            self._busy = True
        self._state('stopping', 'Stopping playback and pipeline nodes...')
        self._queue.put(('stop', None))

    def pause(self):
        with self._lock:
            if self._busy or self.state not in ('running', 'paused'):
                return False
            self._busy = True
        paused = self.state == 'running'
        self._state('pausing', 'Pausing...' if paused else 'Resuming...')
        self._queue.put(('pause', paused))
        return True

    def _check_cancel(self):
        if self._cancel.is_set():
            raise Cancelled()

    @staticmethod
    def _graph():
        graph = master_call('getSystemState')
        nodes = {node for section in graph for _, entries in section for node in entries}
        return nodes, dict(graph[0])

    def _no_conflicts(self):
        """不接管终端或其他 GUI 启动的同名节点，也不抢占其他 /clock 发布者。"""
        nodes, publishers = self._graph()
        conflicts = sorted(nodes & BUSINESS_NODES)
        clocks = publishers.get('/clock', [])
        outputs = ('/ft/detection', '/ft/track', '/ft/target', '/ft/debug/image',
                   '/ft/gui/other/detection', '/ft/gui/other/debug/image', '/ft/debug/comparison',
                   '/ft/debug/prediction_image', '/ft/gui/other/debug/prediction_image',
                   '/ft/debug/prediction_comparison_image')
        conflicts += [name for topic in outputs for name in publishers.get(topic, [])]
        if conflicts or clocks:
            raise RuntimeError('An external session is active. Stop it first: ' + ', '.join(sorted(set(conflicts + clocks))))

    def _wait_nodes(self):
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            self._check_cancel()
            if any(process.process.poll() is not None for process in self.pipelines):
                raise RuntimeError('The pipeline failed to start. See the log below.')
            nodes, _ = self._graph()
            # 启动中也记录已经确认属于本次会话的节点，便于部分失败后的注销检查。
            for process in self.pipelines:
                process.remember()
            alive = {pid for process in self.pipelines for pid in process.alive_pids()}
            for name in self.expected & nodes:
                try:
                    identity = node_identity(name)
                    if identity[1] in alive:
                        self._nodes[name] = identity
                except (OSError, RuntimeError):
                    pass
            if self.expected <= nodes:
                try:
                    for side in self.active.sides:
                        rospy.wait_for_service(side.tuning + '/set_parameters', timeout=0.2)
                    identities = {name: node_identity(name) for name in self.expected}
                    for process in self.pipelines:
                        process.remember()
                    alive = {pid for process in self.pipelines for pid in process.alive_pids()}
                    if not {item[1] for item in identities.values()} <= alive:
                        raise RuntimeError('A registered node is not owned by this session. Check for conflicting sessions.')
                    for name, (side, package) in self._business.items():
                        verify_loaded(side.workspace, package, identities[name][1])
                    self._nodes = identities
                    return
                except (rospy.ROSException, OSError):
                    pass
            self._cancel.wait(0.1)
        raise RuntimeError('Timed out waiting for pipeline nodes or the detector parameter service.')

    def _set_paused(self, paused):
        """服务调用放到独立线程并限制等待时间，失联的播放器不能卡住停止操作。"""
        rospy.wait_for_service(PLAYER + '/pause_playback', timeout=3.0)
        proxy = rospy.ServiceProxy(PLAYER + '/pause_playback', SetBool)
        done, result = threading.Event(), []

        def request():
            try:
                result.append(proxy(paused))
            except Exception as error:
                result.append(error)
            finally:
                done.set()

        threading.Thread(target=request, daemon=True).start()
        deadline = time.monotonic() + 3.0
        try:
            while not done.wait(0.05):
                self._check_cancel()
                if time.monotonic() > deadline:
                    raise RuntimeError('The playback pause service timed out.')
            if isinstance(result[0], Exception):
                raise RuntimeError('Playback service failed: ' + str(result[0]))
            if not result[0].success:
                raise RuntimeError(result[0].message)
        finally:
            proxy.close()

    def _start(self, options):
        session = self.catalog.prepare(options)
        self._check_cancel()
        self._cleanup()
        # 等待上一会话注销节点，避免重启时将自己的旧注册误判为外部进程。
        deadline = time.monotonic() + 3.0
        while True:
            self._check_cancel()
            try:
                self._no_conflicts()
                break
            except RuntimeError:
                if time.monotonic() >= deadline:
                    raise
                self._cancel.wait(0.1)
        self.active = session
        self.session_changed.emit(session)
        shared_launch = self.catalog.root / 'bonus/gui/ft_rqt_gui/launch/side.launch'
        self.expected = set()
        self._business = {}
        for side in session.sides:
            nodes = side.nodes(options)
            self._business.update({name: (side, package) for name, package in nodes.items()})
            self.expected.update(nodes)
            comparison = options.compare and options.tracker and options.selector
            if comparison:
                self.expected.add(side.namespace + '/compare_predictions')
            arguments = ['dataset:=' + options.dataset, 'mode:=' + side.mode,
                         'node_ns:=' + (side.namespace or '/'), 'prefix:=' + side.prefix,
                         'tf_topic:=' + side.tf_topic,
                         'tf_static_topic:=' + ('/tf_static' if side.primary else side.prefix + '/tf_static'),
                         'image_topic:=' + session.image_topic[:-len('/compressed')],
                         'camera_info_topic:=' + session.camera_info_topic,
                         'tracker:=' + str(options.tracker).lower(),
                         'selector:=' + str(options.selector).lower(),
                         'prediction_comparison:=' + str(comparison).lower()]
            self.pipelines.append(OwnedProcess(['roslaunch', str(shared_launch), *arguments],
                side.workspace.root, lambda line, label=side.label: self.log.emit('['+label+'] '+line),
                env=side.workspace.environment))
            self.log.emit('Starting {} ({}) from {}'.format(side.label, side.mode, side.workspace.root))
        # 投影工具始终随选板运行，使用控制台环境，旧工作区无需安装新增工具或消息。
        if options.selector:
            for side in session.sides:
                name = 'ft_gui_overlay_primary' if side.primary else 'ft_gui_overlay_other'
                self.pipelines.append(OwnedProcess(
                    ['rosrun', 'ft_bringup', 'reproject_predictions.py', '__name:=' + name,
                     '_image_topic:=' + side.prefix + '/debug/image',
                     '_camera_info_topic:=' + session.camera_info_topic,
                     '_prediction_topic:=' + side.prefix + '/debug/rotation_prediction',
                     '_target_topic:=' + side.prefix + '/target',
                     '_output_topic:=' + side.prefix + '/debug/prediction_image',
                     '_full_pose:=' + str(side.workspace.prediction_debug).lower()],
                    self.catalog.root, lambda line, label=side.label: self.log.emit('[Projection '+label+'] '+line)))
                self.expected.add('/' + name)
        if options.compare:
            left, right = session.sides
            self.pipelines.append(OwnedProcess(
                ['rosrun', 'ft_rqt_gui', 'compare_images', '__name:=ft_gui_comparison',
                 '_left_topic:=' + left.prefix + '/debug/image', '_right_topic:=' + right.prefix + '/debug/image',
                 '_left_label:=' + left.label + ' (' + left.mode + ')',
                 '_right_label:=' + right.label + ' (' + right.mode + ')'],
                self.catalog.root, lambda line: self.log.emit('[Comparison] ' + line)))
            self.expected.add('/ft_gui_comparison')
            if options.selector:
                self.pipelines.append(OwnedProcess(
                    ['rosrun', 'ft_rqt_gui', 'compare_images', '__name:=ft_gui_prediction_comparison',
                     '_left_topic:=' + left.prefix + '/debug/prediction_image',
                     '_right_topic:=' + right.prefix + '/debug/prediction_image',
                     '_left_label:=' + left.label + ' prediction',
                     '_right_label:=' + right.label + ' prediction',
                     '_output_topic:=/ft/debug/prediction_comparison_image'],
                    self.catalog.root, lambda line: self.log.emit('[Prediction comparison] '+line)))
                self.expected.add('/ft_gui_prediction_comparison')
        if options.rviz:
            self.viewers = Viewers(self.catalog.root, session, self.log.emit)
        self._state('starting', 'Waiting for pipeline nodes to become ready...')
        self._wait_nodes()
        self._check_cancel()
        playback = [str(self.catalog.root / 'run.sh'), 'play', options.dataset, '--quiet', '--pause',
                    '--wait-for-subscribers', '--rate', str(options.rate), '--start', str(options.start),
                    '__name:=' + PLAYER.lstrip('/')]
        if options.loop:
            playback.append('--loop')
        self.player = OwnedProcess(playback, self.catalog.root, lambda line: self.log.emit('[Playback] ' + line))
        rospy.wait_for_service(PLAYER + '/pause_playback', timeout=3.0)
        self._nodes[PLAYER] = node_identity(PLAYER)
        self.player.remember()
        if self._nodes[PLAYER][1] not in self.player.alive_pids():
            raise RuntimeError('The dedicated player name is already used by another process.')
        self._set_paused(False)
        self._missing = 0
        self._last_check = time.monotonic()
        self.log.emit('Started {} / {}; speed {}x, start {} s, loop {}.'.format(
            options.dataset, options.mode, options.rate, options.start, 'on' if options.loop else 'off'))
        self._state('running', 'Playing: ' + options.dataset)

    def _remove_stale_registrations(self):
        """异常退出可能留下注册，只清理 URI 仍相同且已拒绝连接的本次节点。"""
        if not self._nodes:
            return
        try:
            graph = master_call('getSystemState')
        except Exception:
            return  # master 已退出时没有可清理的注册。
        for name, (uri, _) in self._nodes.items():
            try:
                try:
                    current = master_call('lookupNode', name)
                except RuntimeError:
                    continue
                if current != uri:
                    continue
                try:
                    node_identity(name)
                    continue  # 同名 URI 又有活跃服务，不能当作自己的旧节点处理。
                except ConnectionRefusedError:
                    pass
                except RuntimeError:
                    continue
                for method, section in zip(('unregisterPublisher', 'unregisterSubscriber'), graph[:2]):
                    for topic, nodes in section:
                        if name in nodes:
                            master_call(method, topic, uri, caller_id=name)
                for service, nodes in graph[2]:
                    if name in nodes:
                        api = master_call('lookupService', service)
                        master_call('unregisterService', service, api, caller_id=name)
            except Exception as error:
                self.log.emit('Stale registration check: {}: {}'.format(name, error))

    def _cleanup(self):
        """回放先停，业务 launch 后停；清理失败时保留对象，禁止覆盖其所有权。"""
        if self.player:
            self.player.stop()
            self.player = None
        if self.viewers:
            self._nodes.update(self.viewers.identities)
            self.viewers.stop()
            self.viewers = None
        for process in list(reversed(self.pipelines)):
            process.stop()
            self.pipelines.remove(process)
        self._remove_stale_registrations()
        self.active = None
        self.expected.clear()
        self._business.clear()
        self._nodes.clear()
        self.session_changed.emit(None)

    def _monitor(self):
        if self.state not in ('running', 'paused', 'finished'):
            return
        if any(process.process.poll() is not None for process in self.pipelines):
            raise RuntimeError('The pipeline launch process exited. See the log.')
        if self.viewers:
            self.viewers.poll()
        if self.player and self.state != 'finished':
            code = self.player.process.poll()
            if code is not None:
                if code != 0:
                    raise RuntimeError('Playback exited unexpectedly with code {}.'.format(code))
                self._state('finished', 'Playback finished. The last image is retained. Use Apply / Restart to replay.')
        if time.monotonic() - self._last_check >= 1.0:
            self._last_check = time.monotonic()
            nodes, _ = self._graph()
            # master 可能保留异常退出节点的旧注册，同时检查启动时确认过的进程身份。
            alive = {pid for process in self.pipelines for pid in process.alive_pids()}
            missing = (self.expected - nodes) | {name for name in self.expected if self._nodes[name][1] not in alive}
            self._missing = self._missing + 1 if missing else 0
            if self._missing >= 2:
                raise RuntimeError('Pipeline nodes exited: ' + ', '.join(sorted(missing)))

    def _fail(self, error):
        with self._lock:
            self._busy = True
        self._state('stopping', 'An error occurred. Cleaning up this session...')
        self.log.emit('Error: ' + str(error))
        try:
            self._cleanup()
        except Exception as cleanup_error:
            self.log.emit('Cleanup failed: ' + str(cleanup_error))
        self._state('error', str(error))

    def _run(self):
        """所有会话变更串行执行；信号回到 Qt 主线程后才更新控件。"""
        try:
            while True:
                try:
                    command, value = self._queue.get(timeout=0.1)
                except queue.Empty:
                    try:
                        self._monitor()
                    except Exception as error:
                        self._fail(error)
                    continue
                if command == 'shutdown':
                    break
                try:
                    if command == 'start':
                        self._start(value)
                    elif command == 'stop':
                        self._cleanup()
                        self._state('idle', 'Stopped.')
                    elif command == 'pause':
                        self._set_paused(value)
                        self._state('paused' if value else 'running', 'Paused.' if value else 'Playback resumed.')
                except Cancelled:
                    # stop/shutdown 已入队，由它统一清理，避免提前显示 idle 后又执行旧 stop。
                    self._state('stopping', 'Startup cancelled. Cleaning up this session...')
                except Exception as error:
                    self._fail(error)
        finally:
            try:
                self._cleanup()
            except Exception as error:
                self.log.emit('Shutdown cleanup failed: ' + str(error))

    def shutdown(self):
        self._cancel.set()
        self._queue.put(('shutdown', None))
        self._thread.join(timeout=120.0)
