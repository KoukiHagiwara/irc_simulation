#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

class BallDetectorNode(Node):
    def __init__(self):
        super().__init__('ball_detector_node')
        
        # --- 設定 ---
        package_share_directory = get_package_share_directory('irc_simulation')
        model_path = os.path.join(package_share_directory, 'weights', 'best.pt') # weightsフォルダ推奨
        self.REAL_BALL_DIAMETER_CM = 6.8
        self.FOCAL_LENGTH = 718.409779
        
        # モデル読み込み
        self.get_logger().info(f"Loading YOLO model: {model_path}")
        self.model = YOLO(model_path)
        # self.model.to('cuda') # GPUがないPC環境ならコメントアウト推奨

        # ROS2 通信設定
        self.bridge = CvBridge()
        
        # [入力] Gazeboからのカメラ映像を購読
        self.subscription = self.create_subscription(
            Image,
            '/camera_sensor',  # GazeboとBridgeで設定するトピック名
            self.image_callback,
            10)
            
        # [出力] Arduinoへ送るはずだった情報をパブリッシュ (例: "R:150.5")
        self.publisher_ = self.create_publisher(String, '/ball_info', 10)
        self.get_logger().info('Ready to detect balls from Gazebo camera.')

        self.colors = [(0, 0, 255), (255, 0, 0), (0, 255, 255)] # R, B, Y

    def image_callback(self, msg):
        try:
            # ROS画像メッセージをOpenCV形式に変換
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'CV Bridge Error: {e}')
            return

        # YOLO推論
        results = self.model.predict(frame, conf=0.5, verbose=False)
        
        detected_balls = []
        for res in results:
            boxes = res.boxes.cpu().numpy()
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                pixel_diameter = ((x2 - x1) + (y2 - y1)) / 2.0
                
                distance_cm = 0
                if pixel_diameter > 0:
                    distance_cm = (self.REAL_BALL_DIAMETER_CM * self.FOCAL_LENGTH) / pixel_diameter
                
                detected_balls.append({
                    'distance': distance_cm, 
                    'box': (x1, y1, x2, y2), 
                    'class_id': int(box.cls[0])
                })

        # 最も近いボールを選定
        command_code = 'N'
        dist_val = 0.0

        if detected_balls:
            nearest_ball = min(detected_balls, key=lambda b: b['distance'])
            class_id = nearest_ball['class_id']
            dist_val = nearest_ball['distance']

            # コマンド決定 (0:R, 1:B, 2:Y と仮定)
            if class_id == 0: command_code = 'R'
            elif class_id == 1: command_code = 'B'
            elif class_id == 2: command_code = 'Y'

            # 描画
            x1, y1, x2, y2 = nearest_ball['box']
            cv2.rectangle(frame, (x1, y1), (x2, y2), self.colors[class_id % 3], 2)
            cv2.putText(frame, f"{command_code}:{dist_val:.1f}cm", (x1, y1-10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

        # 情報を送信 (String型: "コマンド:距離")
        msg_str = String()
        msg_str.data = f"{command_code}:{dist_val:.2f}"
        self.publisher_.publish(msg_str)
        self.get_logger().info(f"Published: {msg_str.data}")

        # 確認用画面表示
        cv2.imshow('Gazebo YOLO View', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = BallDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
