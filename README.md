# simulation_irc
知能ロボットコンテスト用シミュレーションパッケージ
- Gazebo (Ignition Fortress) を使用して、ロボットの動作検証を行うためのROS 2パッケージです。
- Gazebo Classicのサポート終了に伴い、長期間の運用を見据えてFortressを採用しました。

## Nodes
1. ball_color_node.py
- YOLOv8を用いてカメラ映像からボールを検出するノード
  - 入力: /camera_sensor (sensor_msgs/Image)
  - 出力: /ball_info (std_msgs/String)
  - 処理: 一番手前にあるボールの色と距離を判定し、後段の制御ノードへ送信

2. ball_chaser.cpp
- 実機のArduino制御部分をシミュレーション用に代替した制御ノード
  - 入力: /ball_info (std_msgs/String)
  - 出力: /cmd_vel (geometry_msgs/Twist)
  - 処理: 受け取ったボール情報に基づいてモータ指令（速度・旋回）を生成し、一定距離まで近づくと停止

## Launch Files
シミュレーション環境やオブジェクトを生成するためのスクリプト
- irc.launch.py: ロボットとステージを含むシミュレーション全体を起動
- spawn_random_balls.launch.py: ステージ上にボールをランダムに配置



## 実行画面
<img width="1002" height="874" alt="Image" src="https://github.com/user-attachments/assets/51249a96-f85f-4349-bff8-849432b33e9a" />

## 実行方法
以下の手順でシミュレーションを実行します。

1. シミュレーションの起動
```
$ ros2 launch irc_simulation irc.launch.py
```
2. 画像認識ノードの起動
```
$ ros2 run irc_simulation ball_color_node.py
```
3. 制御ノードの起動
```
$ ros2 run irc_simulation ball_chaser
```
4. 手動操作(ボール探索用)ロボットをボールが見える位置までキーボードで移動させるために使用します。
```
$ ign topic -e -t /keyboard/keypress
```
# 動作環境
- Python 3.10, C++
- Ubuntu 22.04 LTS
- ROS2 humble
- Gazebo Fortress
