#!/usr/bin/env bash
# 加载已构建的本地 ROS overlay，使用项目 Python 执行功能闭环测试。
set -eo pipefail
TEST_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$TEST_ROOT"
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    source /opt/ros/jazzy/setup.bash
fi
if [[ -f install/local_setup.bash ]]; then
    source install/local_setup.bash
fi
export QT_QPA_PLATFORM=offscreen
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_DOMAIN_ID=231
unset ROS_STATIC_PEERS ROS_DISCOVERY_SERVER ROS_LOCALHOST_ONLY
exec "$TEST_ROOT/.venv/bin/python" tests/run.py "$@"
