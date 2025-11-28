# simulation_irc
知能ロボットコンテスト
- gazebo(igniton fortress)によるシミュレーションを行う
- gazebo classicはサポート期間が終了しており、今後もシミュレーションを行うことを考えfortressを使うことにした

## Node説明

- irc.launch.py
シミュレーションを起動させる
- ball_color_node.py
YOLOを使用したカメラによる画像認識を行う
 - 一番手前に映るボールの色とカメラからの距離を判定
- ball_chaaser.cpp
実機でArduinoを使用する部分の代替用のC++のコード
 - 受け取ったボール情報に対してモータを回しボールへ近づく
 - 一定距離まで近づいたらモータを止める
- spawn_random_balls.launch.py
ステージ上にボールを配置



## 実行画面
<img width="1002" height="874" alt="Image" src="https://github.com/user-attachments/assets/51249a96-f85f-4349-bff8-849432b33e9a" />

## 実行方法
実行は以下のコマンドを用いて行います。４つ目のターミナル操作はボールが見える位置まで移動するためのものです。

1つ目のターミナル
```
$ ros2 launch irc_simulation irc.launch.py
```
2つ目のターミナル
```
$ ros2 run irc_simulation ball_color_node.py
```
3つ目のターミナル
```
$ ros2 run irc_simulation ball_chaser
```
4つ目のターミナル
```
$ ign topic -e -t /keyboard/keypress
```
# 動作環境
- Python 3.10
- Ubuntu 22.04 LTS
- ROS2 humble
- Gazebo Fortress
