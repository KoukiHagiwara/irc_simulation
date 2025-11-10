# simulation_irc
知能ロボットコンテスト
- gazebo(igniton fortress)によるシミュレーションを行う
- gazebo classicはサポート期間が終了しており、今後もシミュレーションを行うことを考えfortressを使うことにした

## 実行画面
<img width="1002" height="874" alt="Image" src="https://github.com/user-attachments/assets/51249a96-f85f-4349-bff8-849432b33e9a" />

## 実行方法
実行は以下のコマンドを用いて行います。

```
$ ros2 launch irc_simulation irc.launch.py
```

# 動作環境
- Python 3.10
- Ubuntu 22.04 LTS
- ROS2 humble
- Gazebo Fortress
