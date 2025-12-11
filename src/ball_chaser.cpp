#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <vector>
#include <sstream>
#include <cmath> 

enum class RobotState {
    LINE_TRACING,
    CHASING,
    COLLECTING,
    RETURNING
};

class BallChaser : public rclcpp::Node {
public:
    BallChaser() : Node("ball_chaser") {
        sub_ball_ = this->create_subscription<std_msgs::msg::String>(
            "/ball_info", 10, std::bind(&BallChaser::ball_callback, this, std::placeholders::_1));
        
        // ★5つのセンサー購読
        sub_ll_ = this->create_subscription<sensor_msgs::msg::Image>("/line/left_left", 10, [this](const sensor_msgs::msg::Image::SharedPtr msg){ val_ll_ = msg->data[0]; });
        sub_l_  = this->create_subscription<sensor_msgs::msg::Image>("/line/left", 10,      [this](const sensor_msgs::msg::Image::SharedPtr msg){ val_l_  = msg->data[0]; });
        sub_c_  = this->create_subscription<sensor_msgs::msg::Image>("/line/center", 10,    [this](const sensor_msgs::msg::Image::SharedPtr msg){ val_c_  = msg->data[0]; });
        sub_r_  = this->create_subscription<sensor_msgs::msg::Image>("/line/right", 10,     [this](const sensor_msgs::msg::Image::SharedPtr msg){ val_r_  = msg->data[0]; });
        sub_rr_ = this->create_subscription<sensor_msgs::msg::Image>("/line/right_right", 10,[this](const sensor_msgs::msg::Image::SharedPtr msg){ val_rr_ = msg->data[0]; });

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        current_state_ = RobotState::LINE_TRACING;
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), std::bind(&BallChaser::control_loop, this));
            
        RCLCPP_INFO(this->get_logger(), "Ball Chaser (5 Sensors) Started.");
    }

private:
    void ball_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::string data = msg->data;
        char cmd_buf[10]; 
        if (sscanf(data.c_str(), "%[^:]:%lf:%lf", cmd_buf, &ball_dist_, &ball_center_x_) >= 1) {
            ball_command_ = cmd_buf[0];
        } else {
            ball_command_ = 'N';
        }
    }

    void control_loop() {
        auto twist = geometry_msgs::msg::Twist();
        auto now = this->now();
        
        const double TARGET_DISTANCE = 50.0;
        const double MAX_CHASE_DISTANCE = 70.0;
        const int BLACK_THRESHOLD = 120;
        
        const double TRACE_SPEED = 0.15;
        const double TURN_SPEED_WEAK = 0.5; // 内側センサー用
        const double TURN_SPEED_STRONG = 0.9; // 外側センサー用 (急旋回)

        std::string debug_state = "STOP";

        switch (current_state_) {
            case RobotState::LINE_TRACING:
                if (ball_command_ != 'N' && ball_dist_ <= MAX_CHASE_DISTANCE) {
                    current_state_ = RobotState::CHASING;
                    RCLCPP_INFO(this->get_logger(), "Ball Found! Switch to CHASING.");
                    return;
                }

                {
                    // 5つのセンサー判定
                    bool is_ll = (val_ll_ < BLACK_THRESHOLD);
                    bool is_l  = (val_l_  < BLACK_THRESHOLD);
                    bool is_c  = (val_c_  < BLACK_THRESHOLD);
                    bool is_r  = (val_r_  < BLACK_THRESHOLD);
                    bool is_rr = (val_rr_ < BLACK_THRESHOLD);

                    // 優先順位: 外側検知 -> 内側検知 -> 中央検知
                    if (is_ll) {
                        // 左端が黒 -> 大きく左へ
                        twist.linear.x = TRACE_SPEED * 0.3; // 減速
                        twist.angular.z = TURN_SPEED_STRONG; 
                        debug_state = "Turn Left STRONG";
                    }
                    else if (is_rr) {
                        // 右端が黒 -> 大きく右へ
                        twist.linear.x = TRACE_SPEED * 0.3;
                        twist.angular.z = -TURN_SPEED_STRONG;
                        debug_state = "Turn Right STRONG";
                    }
                    else if (is_l) {
                        // 左が黒 -> 普通に左へ
                        twist.linear.x = TRACE_SPEED * 0.5;
                        twist.angular.z = TURN_SPEED_WEAK;
                        debug_state = "Turn Left";
                    }
                    else if (is_r) {
                        // 右が黒 -> 普通に右へ
                        twist.linear.x = TRACE_SPEED * 0.5;
                        twist.angular.z = -TURN_SPEED_WEAK;
                        debug_state = "Turn Right";
                    }
                    else if (is_c) {
                        // 中央のみ黒 -> 直進
                        twist.linear.x = TRACE_SPEED;
                        twist.angular.z = 0.0;
                        debug_state = "Go Straight";
                    }
                    else {
                        // ロスト -> 停止
                        twist.linear.x = 0.0;
                        twist.angular.z = 0.0;
                        debug_state = "Lost";
                    }
                }
                break;

            case RobotState::CHASING:
                // (元の追跡ロジック)
                if (ball_command_ == 'N' || ball_dist_ > MAX_CHASE_DISTANCE) {
                    twist.linear.x = 0.0; 
                } else {
                    double error = 320.0 - ball_center_x_;
                    twist.angular.z = 0.005 * error;
                    if (std::abs(error) < 50.0) {
                        if (ball_dist_ > TARGET_DISTANCE) {
                            twist.linear.x = 0.2; 
                        } else {
                            twist.linear.x = 0.0;
                            current_state_ = RobotState::COLLECTING;
                            state_start_time_ = now;
                            RCLCPP_INFO(this->get_logger(), "Reached! Start Collecting.");
                        }
                    }
                }
                break;

            case RobotState::COLLECTING:
                twist.linear.x = 0.0;
                if ((now - state_start_time_).seconds() > 5.0) {
                    current_state_ = RobotState::RETURNING;
                    state_start_time_ = now;
                    RCLCPP_INFO(this->get_logger(), "Done. Returning...");
                }
                break;

            case RobotState::RETURNING:
                twist.linear.x = -0.2;
                if ((now - state_start_time_).seconds() > 3.0) {
                    current_state_ = RobotState::LINE_TRACING;
                    RCLCPP_INFO(this->get_logger(), "Returned. Back to Line Trace.");
                }
                break;
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
            "[%s] S:[%3d %3d %3d %3d %3d] -> Lin:%.2f Ang:%.2f", 
            debug_state.c_str(), 
            val_ll_, val_l_, val_c_, val_r_, val_rr_,
            twist.linear.x, twist.angular.z);

        publisher_->publish(twist);
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_ball_;
    // 5つのセンサー
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_ll_, sub_l_, sub_c_, sub_r_, sub_rr_;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    RobotState current_state_;
    rclcpp::Time state_start_time_;
    
    // センサー値
    uint8_t val_ll_ = 255, val_l_ = 255, val_c_ = 255, val_r_ = 255, val_rr_ = 255;
    
    char ball_command_ = 'N';
    double ball_dist_ = 0.0;
    double ball_center_x_ = 320.0;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BallChaser>());
    rclcpp::shutdown();
    return 0;
}