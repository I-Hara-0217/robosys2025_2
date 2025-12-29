#!/bin/bash
# SPDX-FileCopyrightText: 2025 Ibuki Hara
# SPDX-License-Identifier: BSD-3-Clause

ng () {
    res=1
}

res=0

export PYTHONUNBUFFERED=1
export RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED=1

source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --packages-select saying_sender
source install/setup.bash

timeout 25s ros2 run saying_sender quotes_publisher > /tmp/saying_sender.log 2>&1 || true

count=$(grep -c "Publish:" /tmp/saying_sender.log)

[ "$count" -ge 3 ] || ng "$LINENO"

[ "$res" = 0 ] && echo OK

exit $res
