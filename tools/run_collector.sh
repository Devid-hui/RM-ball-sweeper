#!/bin/sh
# 先设置 SDK 的运行库搜索路径，再启动 Python 进程。
set -eu
project_dir=$(CDPATH='' cd -- "$(dirname -- "$0")/.." && pwd)
build_dir=${BALL_COLLECTOR_BUILD_DIR:-"$project_dir/build-hik"}
sdk_dir="$project_dir/io/hikrobot/lib/arm64"

export PYTHONPATH="$build_dir:$project_dir${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="$sdk_dir${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "${BALL_COLLECTOR_PYTHON:-python3}" -m tasks.ball_collect.run "$@"
