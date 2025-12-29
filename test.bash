#!/bin/bash
# SPDX-FileCopyrightText: 2025 Ibuki Hara
# SPDX-License-Identifier: BSD-3-Clause

# 不具合が出たらすぐに停止するようにします
set -e

res=0
export PYTHONUNBUFFERED=1
export RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED=1

# OS側の設定読み込み
source /opt/ros/jazzy/setup.bash

# 作業場へ移動して組み立て
cd ~/ros2_ws
colcon build --packages-select saying_sender

# 新しく作った部品の設定を「絶対住所」で読み込み
source ~/ros2_ws/install/setup.bash

# 実行して記録を取る（少し長めに待ちます）
# ここで失敗しても停止しないように || true をつけます
set +e
timeout 40s ros2 run saying_sender quotes_publisher > /tmp/saying_sender.log 2>&1 || true

# 記録を画面に出して確認
echo "--- 動作の全記録を表示します ---"
cat /tmp/saying_sender.log
echo "------------------------------"

# 送信の合図を数える
count=$(grep -c "Publish:" /tmp/saying_sender.log)
echo "確認された送信回数: $count"

# 判定（3回以上あれば合格）
if [ "$count" -ge 3 ]; then
    echo "検証結果：合格"
    res=0
else
    echo "検証結果：不合格（送信回数が足りません）"
    res=1
fi

exit $res
