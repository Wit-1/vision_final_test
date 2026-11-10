"""Resolve each workspace in a fresh ROS environment and verify loaded binaries."""

from dataclasses import dataclass
import json
import os
from pathlib import Path
import re
import subprocess


@dataclass(frozen=True)
class Workspace:
    root: Path
    environment: dict
    executables: dict
    signatures: dict


def environment_for(root):
    """只加载 Noetic 与指定工作区，避免继承另一版本的 Python 模块和动态库。"""
    root = Path(root).expanduser().resolve()
    if not (root / 'devel/setup.bash').is_file():
        raise ValueError('Workspace is not built. Run ./run.sh build there first: ' + str(root))
    env = dict(os.environ)
    for name in ('CMAKE_PREFIX_PATH', 'ROS_PACKAGE_PATH', 'ROSLISP_PACKAGE_DIRECTORIES',
                 'PYTHONPATH', 'LD_LIBRARY_PATH', 'PKG_CONFIG_PATH', 'CATKIN_SHELL'):
        env.pop(name, None)
    env.update(PATH='/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin',
               LANG='C.UTF-8', LC_ALL='C.UTF-8', LANGUAGE='en')
    result = subprocess.run(
        ['bash', '--noprofile', '--norc', '-c',
         'source /opt/ros/noetic/setup.bash && source "$1/devel/setup.bash" && env -0',
         'workspace-environment', str(root)], env=env, capture_output=True, timeout=10)
    if result.returncode:
        raise ValueError('Cannot load workspace environment: ' + result.stderr.decode(errors='replace'))
    return {key: value for key, value in
            (item.decode().split('=', 1) for item in result.stdout.split(b'\0') if b'=' in item)}


def inspect_workspace(root, dataset, needs_nn, tracker, selector):
    root = Path(root).expanduser().resolve()
    env = environment_for(root)
    probe = Path(__file__).resolve().parents[2] / 'scripts/workspace_probe'
    result = subprocess.run(
        ['python3', str(probe), str(root), dataset, str(int(needs_nn)),
         str(int(tracker)), str(int(selector))], env=env, capture_output=True, text=True, timeout=15)
    if result.returncode:
        raise ValueError('Workspace check failed for {}: {}'.format(root, result.stderr.strip()))
    info = json.loads(result.stdout)
    return Workspace(root, env, info['executables'], info['signatures'])


def verify_loaded(workspace, package, pid):
    """从 /proc 检查实际程序与项目动态库，而非仅相信 ROS 节点名。"""
    executable = Path('/proc/{}/exe'.format(pid)).resolve()
    if executable != Path(workspace.executables[package]):
        raise RuntimeError('Wrong executable for {}: {}'.format(package, executable))
    maps = Path('/proc/{}/maps'.format(pid)).read_text()
    libraries = set()
    for line in maps.splitlines():
        fields = line.split(None, 5)
        if len(fields) == 6 and '/libft_' in fields[5]:
            decoded = re.sub(r'\\([0-7]{3})', lambda m: chr(int(m[1], 8)), fields[5])
            path = Path(decoded).resolve()
            if workspace.root not in path.parents or not path.is_file():
                raise RuntimeError('A project library came from another workspace: ' + str(path))
            libraries.add(path.name)
    expected = {'ft_detector': 'libft_detector_core.so', 'ft_tracker': 'libft_tracker_core.so',
                'ft_selector': 'libft_selector_core.so'}
    if expected[package] not in libraries:
        raise RuntimeError('Cannot verify the loaded algorithm library for ' + package)
