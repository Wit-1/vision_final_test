"""Bounded ROS master calls and cleanup restricted to processes we started."""

import os
from pathlib import Path
import signal
import subprocess
import threading
import time
from xmlrpc.client import ServerProxy, Transport


class _Transport(Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 2.0
        return connection


def master_call(method, *args, caller_id='/ft_rqt_gui'):
    """每次使用独立连接，避免后台检查共享 XML-RPC 连接或无限等待。"""
    uri = os.environ.get('ROS_MASTER_URI', 'http://localhost:11311')
    with ServerProxy(uri, transport=_Transport()) as proxy:
        code, message, value = getattr(proxy, method)(caller_id, *args)
    if code != 1:
        raise RuntimeError(message)
    return value


def node_identity(name):
    uri = master_call('lookupNode', name)
    with ServerProxy(uri, transport=_Transport()) as proxy:
        code, message, pid = proxy.getPid('/ft_rqt_gui')
    if code != 1:
        raise RuntimeError(message)
    return uri, pid


def _process_table():
    """PID 配合启动时刻识别进程；启动时刻可防止误伤被复用的 PID。"""
    result = {}
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            fields = (entry / 'stat').read_text().rsplit(')', 1)[1].split()
            result[int(entry.name)] = (int(fields[1]), fields[19], fields[0])
        except (OSError, IndexError, ValueError):
            pass
    return result


class OwnedProcess:
    """记录一个子进程及其后代，正常关闭失败时只清理这些已确认的进程。"""

    def __init__(self, command, cwd, log=None, env=None):
        self.command = command
        self.process = subprocess.Popen(
            command, cwd=str(cwd), env=env, start_new_session=True,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE if log else None,
            stderr=subprocess.STDOUT if log else None,
            text=True, encoding='utf-8', errors='replace', bufsize=1,
        )
        table = _process_table()
        self.identities = ({self.process.pid: table[self.process.pid][1]}
                           if self.process.pid in table else {})
        self.remember()
        self.reader = None
        if log:
            self.reader = threading.Thread(target=self._read, args=(log,), daemon=True)
            self.reader.start()

    def _read(self, log):
        for line in self.process.stdout:
            if line.strip():
                log(line.rstrip())

    def remember(self):
        table = _process_table()
        roots = set()
        for pid, start in self.identities.items():
            if pid in table and table[pid][1] == start:
                roots.add(pid)
        pending = set(roots)
        while pending:
            children = {pid for pid, item in table.items() if item[0] in pending} - roots
            roots.update(children)
            pending = children
        for pid in roots:
            if pid in table:
                self.identities[pid] = table[pid][1]

    def alive_pids(self):
        table = _process_table()
        return [pid for pid, start in self.identities.items()
                if pid in table and table[pid][1] == start and table[pid][2] != 'Z']

    def _wait(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.remember()
            self.process.poll()
            if not self.alive_pids():
                return True
            time.sleep(0.05)
        return False

    def stop(self, grace_period=20.0):
        """先让 roslaunch 自行回收节点，超时再逐个终止已记录的后代。"""
        self.remember()
        if self.process.poll() is None:
            try:
                os.kill(self.process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        else:
            # 窗口被强制关闭时父进程已消失，仍先正常通知记录中的后代退出。
            for pid in self.alive_pids():
                try:
                    os.kill(pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
        if not self._wait(grace_period):
            for sig, delay in ((signal.SIGTERM, 2.0), (signal.SIGKILL, 2.0)):
                for pid in self.alive_pids():
                    try:
                        os.kill(pid, sig)
                    except ProcessLookupError:
                        pass
                if self._wait(delay):
                    break
        self.process.poll()
        if self.reader:
            self.reader.join(timeout=1.0)
            if not self.reader.is_alive():
                self.process.stdout.close()
        if self.alive_pids():
            raise RuntimeError('Owned child processes are still running. A new session cannot start yet.')
