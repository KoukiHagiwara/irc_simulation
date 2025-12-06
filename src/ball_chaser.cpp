#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <string>
#include <vector>
#include <sstream>
#include <cmath> 

// ロボットの状態定義
enum class RobotState {
    CHASING,    // ボール追跡中
    COLLECTING, // ボール回収中（5秒停止）
    RETURNING   // ライン復帰中（逆走）
};

class BallChaser : public rclcpp::Node {
public:
    BallChaser() : Node("ball_chaser") {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/ball_info", 10, std::bind(&BallChaser::topic_callback, this, std::placeholders::_1));
        
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // 初期状態設定
        current_state_ = RobotState::CHASING;
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
        // --- 1. データ受信と解析 ---
        std::string data = msg->data;
        char command = 'N';
        double distance = 0.0;
        double center_x = 320.0;
        
        char cmd_buf[10]; 
        if (sscanf(data.c_str(), "%[^:]:%lf:%lf", cmd_buf, &distance, &center_x) >= 1) {
            command = cmd_buf[0];
        }

        auto twist = geometry_msgs::msg::Twist();

        // --- 2. パラメータ設定 ---
        // ★変更点: 停止(回収開始)する距離を 30.0 -> 50.0 に変更
        const double TARGET_DISTANCE = 50.0;   
        
        // ★追加点: 追跡を開始する最大距離 (これより遠いと無視)
        const double MAX_CHASE_DISTANCE = 70.0;

        const double CENTER_THRESHOLD = 50.0; 
        const double KP_ANGULAR = 0.005;
        const double LINEAR_SPEED = 0.2;
        const double REVERSE_SPEED = -0.2;     // 逆走速度

        // 時間計測用
        auto now = this->now();

        // --- 3. ステートマシン（状態ごとの動作） ---
        
        switch (current_state_) {
            // ==========================================
            // 状態A: ボールを追いかけている時
            // ==========================================
            case RobotState::CHASING:
                // ボールが見えない時は停止（キーボード操作用）
                if (command == 'N') {
                    return; 
                }

                // ★追加ロジック: 距離が70cmより遠い場合も無視する（キーボード操作用）
                if (distance > MAX_CHASE_DISTANCE) {
                    return;
                }

                // ここから先は「見えている」かつ「70cm以内」
                
                // 旋回制御
                {
                    double error = 320.0 - center_x;
                    twist.angular.z = KP_ANGULAR * error;

                    // 前進制御
                    if (std::abs(error) < CENTER_THRESHOLD) {
                        if (distance > TARGET_DISTANCE) {
                            // 50cmより遠いので近づく
                            twist.linear.x = LINEAR_SPEED; 
                        } else {
                            // ★目標距離(50cm)以下になった！ -> 回収モードへ移行
                            twist.linear.x = 0.0;
                            current_state_ = RobotState::COLLECTING;
                            state_start_time_ = now; // 時間計測開始
                            RCLCPP_INFO(this->get_logger(), "Ball Reached (<50cm)! Starting Collection (5s)...");
                        }
                    }
                }
                break;

            // ==========================================
            // 状態B: ボール回収中（5秒間停止）
            // ==========================================
            case RobotState::COLLECTING:
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;

                // 5秒経過したかチェック
                if ((now - state_start_time_).seconds() > 5.0) {
                    // 5秒経ったら -> 復帰モードへ移行
                    current_state_ = RobotState::RETURNING;
                    state_start_time_ = now; // 再度、時間計測開始
                    RCLCPP_INFO(this->get_logger(), "Collection Done. Returning to Line (3s)...");
                }
                break;

            // ==========================================
            // 状態C: ライン復帰中（逆走）
            // ==========================================
            case RobotState::RETURNING:
                // 画像は見ずに、ひたすらバックする
                twist.linear.x = REVERSE_SPEED;
                twist.angular.z = 0.0;

                // 3秒間バックする
                if ((now - state_start_time_).seconds() > 3.0) {
                    // バック完了 -> 再び探索モードへ
                    twist.linear.x = 0.0;
                    current_state_ = RobotState::CHASING;
                    RCLCPP_INFO(this->get_logger(), "Returned. Resume Searching.");
                }
                break;
        }

        publisher_->publish(twist);
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    
    // 状態管理用変数
    RobotState current_state_;
    rclcpp::Time state_start_time_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BallChaser>());
    rclcpp::shutdown();
    return 0;
}