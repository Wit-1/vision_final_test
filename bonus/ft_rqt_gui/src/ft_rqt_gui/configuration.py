"""Turn the selected dataset and comparison into one or two isolated sides."""

from copy import deepcopy
from dataclasses import dataclass
import math
import os
from pathlib import Path

import rosbag
import rospkg
import yaml

from .workspace import Workspace, inspect_workspace


@dataclass(frozen=True)
class Options:
    dataset: str = 'blue_1'
    mode: str = 'tradition'
    comparison: str = 'off'
    tracker: bool = True
    selector: bool = True
    rate: float = 0.2
    start: float = 0.0
    loop: bool = False
    rviz: bool = False
    compare_rviz: bool = False
    baseline: str = ''
    candidate: str = ''

    @property
    def compare(self):
        return self.comparison != 'off'


@dataclass(frozen=True)
class Side:
    workspace: Workspace
    mode: str
    label: str
    primary: bool

    @property
    def namespace(self):
        return '' if self.primary else '/ft_gui_other'

    @property
    def prefix(self):
        return '/ft' if self.primary else '/ft/gui/other'

    @property
    def tf_topic(self):
        return '/tf' if self.primary else self.prefix + '/tf'

    @property
    def tuning(self):
        return self.namespace + '/ft_detector/tuning'

    def nodes(self, options):
        packages = ['ft_detector'] + (['ft_tracker'] if options.tracker else []) + (
            ['ft_selector'] if options.selector else [])
        return {self.namespace + '/' + name: name for name in packages}


@dataclass(frozen=True)
class Session:
    options: Options
    bag: Path
    duration: float
    image_topic: str
    camera_info_topic: str
    sides: tuple

    @property
    def primary(self):
        return next(side for side in self.sides if side.primary)

    @property
    def debug_topic(self):
        return self.primary.prefix + '/debug/image'


def project_root():
    explicit = os.environ.get('FT_RQT_GUI_PROJECT_ROOT')
    candidates = [Path(explicit)] if explicit else Path(rospkg.RosPack().get_path('ft_bringup')).parents
    for root in candidates:
        if (root / 'run.sh').is_file() and (root / 'src/ft_bringup/config').is_dir():
            return root.resolve()
    raise RuntimeError('Project root not found. Start the console with ./run.sh gui from the project.')


def _merge(destination, source):
    for key, value in source.items():
        if isinstance(value, dict) and isinstance(destination.get(key), dict):
            _merge(destination[key], value)
        else:
            destination[key] = deepcopy(value)


class Catalog:
    def __init__(self, root):
        self.root = Path(root).resolve()
        self.config = self.root / 'src/ft_bringup/config'
        self.profiles = {p.stem: p for p in sorted((self.config / 'datasets').glob('*.yaml'))}
        if not self.profiles:
            raise RuntimeError('No dataset YAML profiles were found.')

    def prepare(self, options):
        """控制台只提供一份输入；每侧保留自己工作区内的算法与配置。"""
        if (options.dataset not in self.profiles or options.mode not in ('tradition', 'nn') or
                options.comparison not in ('off', 'modes', 'workspaces')):
            raise ValueError('Select a valid dataset, detection mode and comparison type.')
        if not math.isfinite(options.rate) or options.rate <= 0:
            raise ValueError('Playback speed must be a positive number.')
        if not math.isfinite(options.start) or options.start < 0:
            raise ValueError('Start time must be a nonnegative number of seconds.')
        if options.compare_rviz and (not options.compare or not options.rviz):
            raise ValueError('Compare in RViz requires a comparison and Open RViz.')
        config = {}
        for path in (self.config / 'detector.yaml', self.config / 'model.yaml', self.profiles[options.dataset]):
            data = yaml.safe_load(path.read_text())
            if not isinstance(data, dict):
                raise ValueError('Configuration must be a YAML mapping: ' + str(path))
            _merge(config, data)
        bag_path = self.root / 'task' / config['bag_file']
        if not bag_path.is_file():
            raise ValueError('Bag file not found: ' + str(bag_path))
        raw = config['image_topic'] + '/compressed'
        with rosbag.Bag(str(bag_path)) as bag:
            duration = bag.get_end_time() - bag.get_start_time()
            topics = bag.get_type_and_topic_info()[1]
        if options.start >= duration:
            raise ValueError('Start time is outside the bag. Its duration is approximately {:.2f} s.'.format(duration))
        if raw not in topics or config['camera_info_topic'] not in topics:
            raise ValueError('The bag is missing the configured image or CameraInfo topic.')

        def inspect(root, needs_nn):
            return inspect_workspace(root, options.dataset, needs_nn, options.tracker, options.selector)

        # 版本对比各用一套环境；同版模式对比共享资源，但不共享 ROS 节点状态。
        if options.comparison == 'workspaces':
            if not options.baseline.strip() or not options.candidate.strip():
                raise ValueError('Select both Baseline and Candidate workspace directories.')
            baseline = inspect(options.baseline, options.mode == 'nn')
            candidate = inspect(options.candidate, options.mode == 'nn')
            if baseline.root == candidate.root:
                raise ValueError('Baseline and Candidate must be different workspace directories.')
            if baseline.signatures != candidate.signatures:
                raise ValueError('Workspace message interfaces differ. This version requires compatible messages.')
            sides = (Side(baseline, options.mode, 'Baseline', False),
                     Side(candidate, options.mode, 'Candidate', True))
        else:
            workspace = inspect(self.root, options.mode == 'nn' or options.compare)
            if options.comparison == 'modes':
                sides = tuple(Side(workspace, mode, label, mode == options.mode)
                              for mode, label in (('tradition', 'Traditional'), ('nn', 'NN')))
            else:
                sides = (Side(workspace, options.mode, 'Primary', True),)
        return Session(options, bag_path, duration, raw, config['camera_info_topic'], sides)
