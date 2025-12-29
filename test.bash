#!/bin/bash
# SPDX-FileCopyrightText: 2025 Ibuki Hara
# SPDX-License-Identifier: BSD-3-Clause

source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws

# 組み立てを行い、成功するか確認します
colcon build --packages-select saying_sender

# 実行のための設定を読み込みます
source install/setup.bash

# 25秒間実行して、その記録をファイルに保存します
timeout 25s ros2 run saying_sender quotes_publisher > /tmp/saying_sender.log 2>&1 || true

# --- ここから追加：記録された中身を画面に出して確認します ---
echo "--- 動作の記録を確認します ---"
cat /tmp/saying_sender.log
echo "--------------------------"

# 送信された回数を数えます
count=$(grep -c "Publish:" /tmp/saying_sender.log)
echo "送信が確認された回数: $count"

# 判定（3回以上送られていれば合格）
if [ "$count" -ge 3 ]; then
    echo "検証結果：合格"
    exit 0
else
    echo "検証結果：不合格（メッセージが足りません）"
    exit 1
fi
