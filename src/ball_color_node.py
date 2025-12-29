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
        model_path = os.path.join(package_share_directory, 'weights', 'best.pt') 
        self.REAL_BALL_DIAMETER_CM = 6.8
        self.FOCAL_LENGTH = 718.409779
        
        # ★追加設定: 追いかける最大距離 (cm)
        self.MAX_CHASE_DISTANCE = 80.0 

        # モデル読み込み
        self.get_logger().info(f"Loading YOLO model: {model_path}")
        self.model = YOLO(model_path)

        # ROS2 通信設定
        self.bridge = CvBridge()
        
        # [入力]
        self.subscription = self.create_subscription(
            Image,
            '/camera_sensor',
            self.image_callback,
            10)
            
        # [出力]
        self.publisher_ = self.create_publisher(String, '/ball_info', 10)
        self.get_logger().info(f'Ready. Max chase distance set to {self.MAX_CHASE_DISTANCE} cm.')

        self.colors = [(0, 0, 255), (255, 0, 0), (0, 255, 255)] # R, B, Y

        # ターゲット固定用の変数
        self.locked_target_id = None
        self.lost_count = 0

    def image_callback(self, msg):
        try:
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
                center_x = (x1 + x2) / 2.0
                pixel_diameter = ((x2 - x1) + (y2 - y1)) / 2.0
                
                distance_cm = 0
                if pixel_diameter > 0:
                    distance_cm = (self.REAL_BALL_DIAMETER_CM * self.FOCAL_LENGTH) / pixel_diameter
                
                detected_balls.append({
                    'distance': distance_cm, 
                    'center_x': center_x,
                    'box': (x1, y1, x2, y2), 
                    'class_id': int(box.cls[0])
                })

        # --- ターゲット選択ロジック ---
        final_target = None
        
        if self.locked_target_id is not None:
            # 【追跡モード: ロック中のIDと同じボールを探す】
            same_color_balls = [b for b in detected_balls if b['class_id'] == self.locked_target_id]
            
            if same_color_balls:
                final_target = min(same_color_balls, key=lambda b: b['distance'])
                self.lost_count = 0 
            else:
                self.lost_count += 1
                if self.lost_count > 10:
                    self.locked_target_id = None
                    self.lost_count = 0
                    self.get_logger().info("Target Lost. Resetting lock.")
        
        else:
            # 【探索モード: 60cm以下のボールだけを探す】
            # ★変更点: 距離制限フィルターを追加
            valid_balls = [b for b in detected_balls if b['distance'] <= self.MAX_CHASE_DISTANCE]
            
            if valid_balls:
                final_target = min(valid_balls, key=lambda b: b['distance'])
                self.locked_target_id = final_target['class_id'] # ロックオン
                self.get_logger().info(f"Locked on target ID: {self.locked_target_id} (Dist: {final_target['distance']:.1f}cm)")
        

        # --- 結果出力 ---
        command_code = 'N'
        dist_val = 0.0
        center_x_val = 320.0

        if final_target:
            class_id = final_target['class_id']
            dist_val = final_target['distance']
            center_x_val = final_target['center_x']
            
            x1, y1, x2, y2 = final_target['box']
            
            # ★変更点: ターゲットは見つかっているが、距離が60cmより遠い場合の処理
            if dist_val <= self.MAX_CHASE_DISTANCE:
                # 60cm以内なら進むコマンドを出す
                if class_id == 0: command_code = 'R'
                elif class_id == 1: command_code = 'B'
                elif class_id == 2: command_code = 'Y'
                
                # 描画 (有効ターゲットは緑枠)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
                text = f"LOCKED {command_code}:{dist_val:.1f}cm"
                cv2.putText(frame, text, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
            else:
                # ロック中だが60cmより遠くなった -> 停止 ('N')
                # 描画 (範囲外は黄色枠などで区別)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
                text = f"OUT OF RANGE ({dist_val:.1f}cm)"
                cv2.putText(frame, text, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
                # command_code は 'N' のまま

        elif self.locked_target_id is not None:
             cv2.putText(frame, "Searching for locked target...", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

        # 情報を送信
        msg_str = String()
        msg_str.data = f"{command_code}:{dist_val:.2f}:{center_x_val:.2f}"
        self.publisher_.publish(msg_str)

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