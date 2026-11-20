#!/usr/bin/env python3
"""Cache original bag images, benchmark thread settings, and retain data outside Git."""

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import random
import statistics
import subprocess

import rosbag
import yaml


def cache_images(project, directory):
    """保留原 JPEG 和对应标定，每份素材均匀抽取十张；不重新编码图像。"""
    directory.mkdir(parents=True, exist_ok=True)
    cameras = {}
    for dataset in ('red_1_3', 'red_4', 'blue_1', 'blue_guard'):
        config = project / 'src/ft_bringup/config'
        base = yaml.safe_load((config / 'detector.yaml').read_text())
        profile = yaml.safe_load((config / 'datasets' / (dataset + '.yaml')).read_text())
        base.update(profile)
        image_topic = base['image_topic'] + '/compressed'
        info_topic = base['camera_info_topic']
        with rosbag.Bag(str(project / 'task' / base['bag_file'])) as bag:
            start, end = bag.get_start_time(), bag.get_end_time()
            targets = [start + (end-start)*fraction/11 for fraction in range(1, 11)]
            index, info = 0, None
            for topic, message, stamp in bag.read_messages(topics=[image_topic, info_topic]):
                if topic == info_topic:
                    info = message
                    continue
                if index == len(targets):
                    break
                if stamp.to_sec() < targets[index] or info is None:
                    continue
                name = '{}_{:02d}.jpg'.format(dataset, index)
                (directory / name).write_bytes(bytes(message.data))
                cameras[name] = dict(K=list(info.K), D=list(info.D), model=info.distortion_model,
                                     bag=base['bag_file'], offset_s=stamp.to_sec()-start,
                                     stamp_ns=message.header.stamp.to_nsec())
                index += 1
            if index != 10:
                raise RuntimeError('Could not cache ten calibrated frames: ' + dataset)
    (directory / 'cameras.yaml').write_text(yaml.safe_dump(cameras))


def percentile(values):
    values = sorted(values)
    return values[math.ceil(.95*len(values))-1]


def summarize(paths):
    rows = []
    for path in paths:
        with path.open() as source:
            rows.extend(csv.DictReader(source))
    summary = {'samples': len(rows), 'equivalent': all(row['equivalent'] == '1' for row in rows)}
    for field in ('pre_ms', 'infer_ms', 'post_ms', 'total_ms'):
        values = [float(row[field]) for row in rows]
        summary[field] = dict(median=statistics.median(values), mean=statistics.mean(values), p95=percentile(values))
    for field in ('max_tensor_abs', 'max_corner_px', 'max_position_m', 'max_rotation_deg'):
        summary[field] = max(float(row[field]) for row in rows)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project', type=Path, required=True)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--iterations', type=int, default=120)
    parser.add_argument('--warmup', type=int, default=20)
    parser.add_argument('--max-threads', type=int)
    args = parser.parse_args()
    project, output = args.project.resolve(), args.output.resolve()
    if output.exists():
        raise SystemExit('Use a new output directory; existing results are never overwritten.')
    if args.iterations < 1 or args.warmup < 1:
        raise SystemExit('iterations and warmup must be positive')
    output.mkdir(parents=True)
    images, reference = output / 'images', output / 'reference'
    cache_images(project, images)
    count = len(os.sched_getaffinity(0))
    limit = args.max_threads or count
    if limit < 2 or limit > count:
        raise SystemExit('max-threads must be between two and the allowed CPU count')
    model = project / 'assets/models/armor.onnx'
    metadata = dict(platform=platform.platform(), cpu=Path('/proc/cpuinfo').read_text().split('model name')[1].split('\n')[0].strip(': '),
                    affinity=sorted(os.sched_getaffinity(0)), model_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),
                    warmup=args.warmup, iterations=args.iterations, rounds=3,
                    roi=False, reference_threads=2, percentile='nearest rank',
                    equivalence='retained candidate identities/classes/corners and PnP; raw tensor difference diagnostic only')
    metadata['config_sha256'] = {name: hashlib.sha256((project / 'src/ft_bringup/config' / name).read_bytes()).hexdigest()
                                 for name in ('model.yaml', 'detector.yaml')}
    quota = Path('/sys/fs/cgroup/cpu.max')
    metadata['cpu_quota'] = quota.read_text().strip() if quota.exists() else 'cgroup v1 / not exposed'
    (output / 'metadata.json').write_text(json.dumps(metadata, indent=2))

    def run(threads, cv_threads, label, write=False):
        path = output / (label + '.csv')
        command = [str(args.binary.resolve()), '--project', str(project), '--images', str(images),
                   '--reference', str(reference), '--threads', str(threads), '--opencv', str(cv_threads),
                   '--warmup', str(args.warmup), '--iterations', str(args.iterations)]
        if write:
            command.append('--write-reference')
        with path.open('w') as stream:
            subprocess.run(command, stdout=stream, check=True)
        return path

    run(2, 2, 'reference', write=True)
    paths, controls = {}, []
    for round_index in range(3):
        order = list(range(1, limit+1))
        random.Random(20261119 + round_index).shuffle(order)
        controls.append(run(2, 2, 'control_{}_before'.format(round_index)))
        for threads in order:
            label = 'ort_{:02d}_cv_2_round_{}'.format(threads, round_index)
            paths.setdefault((threads, 2), []).append(run(threads, 2, label))
            print('Completed', label, flush=True)
        controls.append(run(2, 2, 'control_{}_after'.format(round_index)))
    summaries = {key: summarize(value) for key, value in paths.items()}
    leaders = sorted((key for key in summaries if summaries[key]['equivalent']),
                     key=lambda key: summaries[key]['infer_ms']['median'])[:3]
    if not leaders:
        raise RuntimeError('No equivalent thread configuration')
    for threads, _ in leaders:
        for cv_threads in (1, 4):
            key = (threads, cv_threads)
            paths[key] = [run(threads, cv_threads, 'ort_{}_cv_{}_round_{}'.format(threads, cv_threads, index))
                          for index in range(3)]
            summaries[key] = summarize(paths[key])
    valid = [key for key in summaries if summaries[key]['equivalent']]
    fastest = min(summaries[key]['infer_ms']['median'] for key in valid)
    close = [key for key in valid if summaries[key]['infer_ms']['median'] <= fastest*1.02]
    best = min(close, key=lambda key: (key[0], summaries[key]['total_ms']['median'], key[1]))
    report = dict(best=dict(ort_threads=best[0], opencv_threads=best[1]),
                  configurations={'ort_{}_cv_{}'.format(*key): value for key, value in summaries.items()},
                  controls=summarize(controls), selection='within 2% of fastest equivalent inference median, then fewer ORT threads and faster total')
    (output / 'summary.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report['best']), flush=True)


if __name__ == '__main__':
    main()
