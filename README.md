# irc_simulation
知能ロボットコンテスト用シミュレーションパッケージ
- Gazebo (Ignition Fortress) を使用して、ロボットの動作検証を行うためのROS 2パッケージです。
- Gazebo Classicのサポート終了に伴い、長期間の運用を見据えてFortressを採用しました。
- 基本的にはライントレースでボールのある場所へ移動してボールを回収したと判定、その後スタート地点へ戻るという一連動作を再現したシュミレーションです。
- カメラとボールの距離が0.7m以下であればライントレースを無視し、ボールへ直進するように切り替わります。また、カメラとボールの距離が0.5m以下であればボールに近づいたと判断しライン上に戻るようにバックして、旋回してからスタート地点へ戻ります。
- 実機ではボールを回収する工程がありますが、ここではボールに対して一定距離近づければボールを回収したと仮定し、回収する個数に対しても1つ回収できたと認識できればスタート地点へ戻ると設定しています。

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
  - 処理: 受け取ったボール情報に基づいてモータ指令（速度・旋回）を生成し、一定距離まで近づくと停止、その後ライン上に戻りスタート地点へ戻る

## Launch Files
シミュレーション環境やオブジェクトを生成するためのスクリプト
- irc.launch.py: ロボットとステージを含むシミュレーション全体を起動
- spawn_random_balls.launch.py: ステージ上にボールをランダムに配置



## 実行画面
<img width="992" height="878" alt="Image" src="https://github.com/user-attachments/assets/f6ac01d1-27c1-4025-872a-d7e2f6010212" />
<video width="1002" height="874" controls src="https://github.com/user-attachments/assets/7161ce11-f42c-4871-b87b-a213896397f7"></video>

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
4. 手動操作(ボール探索用)ロボットをボールが見える位置までキーボードで移動させるために使用します。(使わなくてもシミュレーションできますが、もし制御ノードではなく手動のキーボードで動かしたい場合のみ使います)
```
$ ign topic -e -t /keyboard/keypress
```
## 解析・検証環境
本シミュレーションでは、制御ロジックの妥当性を確認するために、MATLABおよびRViz2を用いた解析を行っています。

### 1. MATLABによる速度追従性の解析
ROS 2の通信データ（rosbag）をMATLABで解析し、PID制御の調整やノイズの影響を確認しています。
- **青線 (/cmd_vel):** プログラムが出力した目標速度
- **赤線 (/odom):** シミュレータ上のロボットの実測速度（意図的にノイズを付加し、実機に近い挙動を再現）

<img width="833" height="733" alt="Image" src="https://github.com/user-attachments/assets/f9155d36-a57e-4af6-a263-a612649f227d" />

### 2. RViz2によるオドメトリの可視化
Gazebo上の挙動だけでなく、ロボットが自己位置をどう認識しているか（オドメトリ）をRViz2上で可視化しています。
- **赤矢印:** ロボットの推定位置と姿勢（/odom）
- Gazebo（右）とRViz2（左）を連携させ、リアルタイムに座標変換(TF)が正しく行われていることを確認しています。
<img width="2114" height="1152" alt="Image" src="https://github.com/user-attachments/assets/0bd5fe66-6eac-4d7f-b81f-20890884616d" />

## 動作環境
- Python 3.10,  C++
- Ubuntu 22.04 LTS
- ROS2 humble
- Gazebo Fortress
