# saying_sender
[![test](https://github.com/I-Hara-0217/saying_sender/actions/workflows/test.yml/badge.svg)](https://github.com/I-Hara-0217/saying_sender/actions/workflows/test.yml)

ROS 2ノードの起動時に、外部のテキストファイルからランダムにメッセージを選択し、パブリッシュするノードを提供するパッケージです。

## 概要
本パッケージは、ロボットの起動時におけるステータス通知や挨拶を想定したパブリッシャーです。 通信の確立を待つため、起動から約2秒後に一度だけメッセージを送信して停止します。

## ノードとトピック
### quotes_publisher
起動後、約2秒の待機時間を経て、以下のトピックへメッセージを1回送信します。
- パブリッシュ先: `quote_topic` [std_msgs/String]
  - 内容: `words.txt` から選ばれた一言を送信します。

## 実行方法
ターミナルで以下のコマンドを実行します。words.txt内のリストから一言が選択され、トピックへパブリッシュされます。

```
$ ros2 run saying_sender quotes_publisher
[INFO] [1767055196.905624720] [quotes_publisher]: Publish: "元気に稼働中です！"
```

## コンフィグレーション
送信されるメッセージの内容は、以下のファイルを編集することでカスタマイズ可能です。
- パス: `~/ros2_ws/src/saying_sender/words.txt`
- フォーマット: プレーンテキスト形式（一行につき一つのメッセージ）

注意:`words.txt`を編集した後は、変更を反映させるため、必ずワークスペースのルートディレクトリで以下のコマンドを実行し、ビルドし直してください。
```
colcon build --packages-select saying_sender
source ~/.bashrc
```

## 必用なソフトウェア
- Python 
    - テスト済み: 3.12
    - 3.12以上

- ROS 2
    - Jazzy Jalisco

- Ubuntu 24.04.1 LTS

## ライセンス
- このソフトウェアパッケージは、3条項BSDライセンス（3-Clause BSD License）の下、再頒布および使用が許可されます。

- © 2025 Ibuki Hara

## 謝辞
- このパッケージの構成や通信の仕組み、テスト方法は、千葉工業大学 ロボットシステム学(2025)の講義資料を参考にしています。
    - [ryuichiueda/slides_marp/robosys2025](https://github.com/ryuichiueda/slides_marp/tree/master/robosys2025)
