#!/usr/bin/env python3
"""Preserve the supplied model, sort graph dependencies and record resource hashes."""

import hashlib
import heapq
import json
from pathlib import Path
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / '.deps/model-tools'))


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def ordered_nodes(graph):
    nodes = list(graph.node)
    sources = {v.name for v in graph.input} | {v.name for v in graph.initializer}
    producers = {}
    for index, node in enumerate(nodes):
        for name in node.output:
            if name in producers or name in sources:
                raise ValueError('Duplicate graph value: ' + name)
            producers[name] = index
    dependencies, children = [], [[] for _ in nodes]
    for index, node in enumerate(nodes):
        missing = [name for name in node.input if name and name not in sources and name not in producers]
        if missing:
            raise ValueError('Missing graph input: ' + ', '.join(missing))
        dependencies.append({producers[name] for name in node.input if name and name not in sources})
        for producer in dependencies[-1]:
            children[producer].append(index)
    ready = [index for index, inputs in enumerate(dependencies) if not inputs]
    heapq.heapify(ready)
    order = []
    while ready:
        index = heapq.heappop(ready)
        order.append(index)
        for consumer in children[index]:
            dependencies[consumer].remove(index)
            if not dependencies[consumer]:
                heapq.heappush(ready, consumer)
    if len(order) != len(nodes):
        raise ValueError('Graph contains a dependency cycle')
    return nodes, order


def main():
    try:
        import onnx
    except ImportError as error:
        raise SystemExit('先在 Noetic 环境运行 ./run.sh setup-inference。') from error
    archive = ROOT / 'task/nn_model.zip'
    model_dir, sample_dir = ROOT / 'assets/models', ROOT / 'assets/samples'
    with zipfile.ZipFile(archive) as source:
        original = source.read('nn_model/armor.onnx')
        samples = {name: source.read('nn_model/' + name) for name in ('sample_1.jpg', 'sample_2.jpg')}
    model = onnx.load_model_from_string(original)
    expected = [('images', [1, 3, 480, 640]), ('output0', [1, 21, 6300])]
    values = list(model.graph.input) + list(model.graph.output)
    if len(values) != 2 or any(v.name != name or
            [d.dim_value for d in v.type.tensor_type.shape.dim] != shape or
            v.type.tensor_type.elem_type != onnx.TensorProto.FLOAT
            for v, (name, shape) in zip(values, expected)):
        raise ValueError('Model interface differs from the supplied README protocol')
    weights = [v.SerializeToString() for v in model.graph.initializer]
    nodes, order = ordered_nodes(model.graph)
    node_bytes = sorted(node.SerializeToString() for node in nodes)
    del model.graph.node[:]
    model.graph.node.extend(nodes[index] for index in order)
    onnx.checker.check_model(model)
    assert weights == [v.SerializeToString() for v in model.graph.initializer]
    assert node_bytes == sorted(node.SerializeToString() for node in model.graph.node)
    derived = model.SerializeToString(deterministic=True)
    model_dir.mkdir(parents=True, exist_ok=True)
    sample_dir.mkdir(parents=True, exist_ok=True)
    original_path = model_dir / 'armor.original.onnx'
    if original_path.exists() and original_path.read_bytes() != original:
        raise ValueError('Existing preserved model differs from task/nn_model.zip')
    original_path.write_bytes(original)
    (model_dir / 'armor.onnx').write_bytes(derived)
    for name, data in samples.items():
        (sample_dir / name).write_bytes(data)
    manifest = {
        'source': 'task/nn_model.zip', 'archive_sha256': sha256(archive.read_bytes()),
        'original_sha256': sha256(original), 'derived_sha256': sha256(derived),
        'operation': 'stable topological node ordering only; tensors and node attributes unchanged',
        'onnx_version': onnx.__version__, 'ir_version': model.ir_version,
        'opsets': {v.domain or 'ai.onnx': v.version for v in model.opset_import},
        'nodes': len(nodes), 'nodes_moved': sum(a != b for a, b in enumerate(order)),
        'input': expected[0], 'output': expected[1],
        'samples_sha256': {name: sha256(data) for name, data in samples.items()},
    }
    (model_dir / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps(manifest, indent=2))
    print('模型：assets/models/armor.onnx；样例：assets/samples/。')


if __name__ == '__main__':
    main()
