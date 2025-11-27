#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <string>
#include <sstream>

// 本来Arduinoで行う制御をここで行う
class BallChaser : public rclcpp::Node {
public:
    BallChaser() : Node("ball_chaser") {
        // Pythonノードからの情報を購読
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/ball_info", 10, std::bind(&BallChaser::topic_callback, this, std::placeholders::_1));
        
        // Gazebo(DiffDrive)への速度指令
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
        // 受信データ形式: "Command:Distance" (例: "R:150.5", "N:0.0")
        std::string data = msg->data;
        char command = data[0];
        double distance = 0.0;

        // 文字列解析 (簡易的)
        if (data.length() > 2) {
            try {
                distance = std::stod(data.substr(2));
            } catch (...) {
                distance = 0.0;
            }
        }

        auto twist = geometry_msgs::msg::Twist();

        // --- Arduinoのloop()内に書くロジックをここに書く ---
        
        if (command == 'N') {
            // ボールが見えない -> 停止（または旋回して探索）
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
        } else {
            // ボールが見えている (R, B, Y いずれか)
            // 例: 指定の距離(30cm)まで近づく
            if (distance > 30.0) {
                twist.linear.x = 0.2; // 前進
                twist.angular.z = 0.0; // 本来は画面中心との偏差で回転制御を入れる
            } else {
                twist.linear.x = 0.0; // 近づいたら停止
            }
            
            // 例: 色によって動作を変えるならここにswitch文など
            if (command == 'R') {
                // 赤なら...
            }
        }

        publisher_->publish(twist);
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BallChaser>());
    rclcpp::shutdown();
    return 0;
}
