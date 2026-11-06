#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
runtime="$project_root/.deps/onnxruntime"
version=1.22.0
digest=8344d55f93d5bc5021ce342db50f62079daf39aaafb5d311a451846228be49b3

if [ "$(uname -s)" != Linux ] || [ "$(uname -m)" != x86_64 ]; then
    echo '此安装入口使用 Linux x86_64 CPU 包；其他平台请自行配置 ONNXRUNTIME_ROOT。' >&2
    exit 1
fi
mkdir -p "$project_root/.deps"
if [ ! -f "$runtime/include/onnxruntime_cxx_api.h" ] || [ ! -f "$runtime/lib/libonnxruntime.so" ]; then
    if [ -e "$runtime" ]; then
        echo "依赖目录不完整，请检查后再运行：$runtime" >&2
        exit 1
    fi
    archive="$(mktemp /tmp/ft-onnxruntime.XXXXXX.tgz)"
    trap 'rm -f -- "$archive"' EXIT
    curl -fsSL --retry 2 --connect-timeout 15 --max-time 180 \
        "https://github.com/microsoft/onnxruntime/releases/download/v$version/onnxruntime-linux-x64-$version.tgz" \
        -o "$archive"
    echo "$digest  $archive" | sha256sum --check --status
    mkdir "$runtime"
    tar -xzf "$archive" --strip-components=1 -C "$runtime"
fi
if [ "$(cat "$runtime/VERSION_NUMBER")" != "$version" ]; then
    echo "本项目使用 ONNX Runtime $version，请检查 $runtime。" >&2
    exit 1
fi

# Local Python tools prepare the model; online inference uses the C++ library above.
if ! PYTHONPATH="$project_root/.deps/model-tools" python3 -c \
    'import onnx,numpy,google.protobuf; assert (onnx.__version__,numpy.__version__,google.protobuf.__version__)==("1.17.0","1.24.4","4.25.3")' \
    2>/dev/null; then
    python3 -m pip install --disable-pip-version-check --upgrade \
        --target "$project_root/.deps/model-tools" \
        onnx==1.17.0 numpy==1.24.4 protobuf==4.25.3
fi
echo "ONNX Runtime $version CPU 与模型准备工具已就绪。"
