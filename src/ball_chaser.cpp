//速度に乱数
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <vector>
#include <sstream>
#include <cmath> 
#include <random>

enum class RobotState {
    LINE_TRACING,
    CHASING,
    COLLECTING,
    RETURNING,
    U_TURN,
    GO_HOME,
    FINISHED
};

class BallChaser : public rclcpp::Node {
public:
    BallChaser() : Node("ball_chaser") {
        sub_ball_ = this->create_subscription<std_msgs::msg::String>(
            "/ball_info", 10, std::bind(&BallChaser::ball_callback, this, std::placeholders::_1));
        
        // 8つのセンサを購読
        auto qos = 10;
        sub_l4_ = this->create_subscription<sensor_msgs::msg::Image>("/line/left4", qos, [this](const sensor_msgs::msg::Image::SharedPtr msg){ val_l4_ = msg->data[0]; });
        sub_l3_ = this->create_subscription<sensor_msgs::msg::Image>("/line/left3", qos, [this](const sensor_msgs::msg::Image::SharedPtr msg){ val_l3_ = msg->data[0]; });
        sub_l2_ = this->create_subscription<sensor_msgs::msg::Image>("/line/left2", qos, [this](const sensor_msgs::msg::Image::SharedPtr msg){ val_l2_ = msg->data[0]; });
        sub_l1_ = this->create_subscription<sensor_msgs::msg::Image>("/line/left1", qos, [this](const sensor_msgs::msg::Image::SharedPtr msg){ val_l1_ = msg->data[0]; });
        
        sub_r1_ = this->create_subscription<sensor_msgs::msg::Image>("/line/right1", qos, [this](const sensor_msgs::msg::Image::SharedPtr msg){ val_r1_ = msg->data[0]; });
        sub_r2_ = this->create_subscription<sensor_msgs::msg::Image>("/line/right2", qos, [this](const sensor_msgs::msg::Image::SharedPtr msg){ val_r2_ = msg->data[0]; });
        sub_r3_ = this->create_subscription<sensor_msgs::msg::Image>("/line/right3", qos, [this](const sensor_msgs::msg::Image::SharedPtr msg){ val_r3_ = msg->data[0]; });
        sub_r4_ = this->create_subscription<sensor_msgs::msg::Image>("/line/right4", qos, [this](const sensor_msgs::msg::Image::SharedPtr msg){ val_r4_ = msg->data[0]; });

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        current_state_ = RobotState::LINE_TRACING;
        
        // 乱数エンジンの初期化
        std::random_device rd;
        generator_ = std::mt19937(rd());
        distribution_ = std::uniform_real_distribution<double>(0.90, 1.10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), std::bind(&BallChaser::control_loop, this));
            
        RCLCPP_INFO(this->get_logger(), "Ball Chaser (8-Sensor Gradient Control) Started.");
    }

private:
    void ball_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::string data = msg->data;
        char cmd_buf[10]; 
        if (sscanf(data.c_str(), "%[^:]:%lf:%lf", cmd_buf, &ball_dist_, &ball_center_x_) >= 1) {
            ball_command_ = cmd_buf[0];
            if (ball_command_ != 'N') {
                last_known_dist_ = ball_dist_;
                last_known_command_ = ball_command_;
            }
        } else {
            ball_command_ = 'N';
        }
    }

    geometry_msgs::msg::Twist apply_motor_noise(geometry_msgs::msg::Twist cmd) {
        if (std::abs(cmd.linear.x) < 0.001 && std::abs(cmd.angular.z) < 0.001) {
            return cmd;
        }
        const double WHEEL_SEPARATION = 0.36; 
        double v = cmd.linear.x;
        double w = cmd.angular.z;
        double v_l = v - (w * WHEEL_SEPARATION / 2.0);
        double v_r = v + (w * WHEEL_SEPARATION / 2.0);
        double noise_l = distribution_(generator_);
        double noise_r = distribution_(generator_);
        v_l *= noise_l;
        v_r *= noise_r;
        geometry_msgs::msg::Twist noisy_cmd;
        noisy_cmd.linear.x = (v_r + v_l) / 2.0;
        noisy_cmd.angular.z = (v_r - v_l) / WHEEL_SEPARATION;
        return noisy_cmd;
    }

    void control_loop() {
        auto twist = geometry_msgs::msg::Twist();
        auto now = this->now();
        
        const double TARGET_DISTANCE = 40.0;
        const double MAX_CHASE_DISTANCE = 70.0;
        const int BLACK_THRESHOLD = 120;
        
        const double TRACE_SPEED = 0.15;
        
        // --- 旋回強度の設定 (グラデーション) ---
        const double TURN_STRONG = 0.9;  // L4, R4
        const double TURN_MEDIUM = 0.5;  // L3, R3
        const double TURN_SLIGHT = 0.25; // L2, R2 (少し曲がる)
        const double TURN_MICRO  = 0.1;  // L1, R1 (微調整)
        
        const double BACK_SPEED = -0.2; 

        std::string debug_state = "STOP";
        
        // 8つのセンサ判定
        bool is_l4 = (val_l4_ < BLACK_THRESHOLD);
        bool is_l3 = (val_l3_ < BLACK_THRESHOLD);
        bool is_l2 = (val_l2_ < BLACK_THRESHOLD);
        bool is_l1 = (val_l1_ < BLACK_THRESHOLD);
        bool is_r1 = (val_r1_ < BLACK_THRESHOLD);
        bool is_r2 = (val_r2_ < BLACK_THRESHOLD);
        bool is_r3 = (val_r3_ < BLACK_THRESHOLD);
        bool is_r4 = (val_r4_ < BLACK_THRESHOLD);

        bool is_any_black = (is_l4 || is_l3 || is_l2 || is_l1 || is_r1 || is_r2 || is_r3 || is_r4);

        int total_balls = red_count_ + blue_count_ + yellow_count_;
        const int GOAL_BALL_COUNT = 1; 

        switch (current_state_) {
            case RobotState::LINE_TRACING:
                if (total_balls >= GOAL_BALL_COUNT) {}

                if (ball_command_ != 'N' && ball_dist_ <= MAX_CHASE_DISTANCE) {
                    current_state_ = RobotState::CHASING;
                    chase_start_time_ = now;
                    last_known_dist_ = ball_dist_; 
                    RCLCPP_INFO(this->get_logger(), "Ball Found! Switch to CHASING.");
                    return;
                }

                // --- ライントレース (行き) ---
                {
                    bool detected_cross_line = (is_l3 && is_r3);

                    if (detected_cross_line) {
                        if (!is_on_cross_line_) {
                            cross_line_count_++;
                            is_on_cross_line_ = true;
                            RCLCPP_INFO(this->get_logger(), "Cross Line: %d", cross_line_count_);
                        }
                    } else {
                        is_on_cross_line_ = false;
                    }

                    if (is_on_cross_line_) {
                        twist.linear.x = TRACE_SPEED;
                        twist.angular.z = 0.0;
                        debug_state = "Ignore Cross Line";
                    }
                    else {
                        // 左側の判定 (カウント2のときは左折禁止)
                        if (is_l4 && cross_line_count_ != 2) { 
                            twist.linear.x = TRACE_SPEED * 0.3; twist.angular.z = TURN_STRONG; debug_state="Left++ (L4)"; 
                        }
                        else if (is_l3 && cross_line_count_ != 2) { 
                            twist.linear.x = TRACE_SPEED * 0.5; twist.angular.z = TURN_MEDIUM; debug_state="Left (L3)"; 
                        }
                        else if (is_l2 && cross_line_count_ != 2) {
                            twist.linear.x = TRACE_SPEED * 0.8; twist.angular.z = TURN_SLIGHT; debug_state="Left (L2)";
                        }
                        else if (is_l1 && cross_line_count_ != 2) {
                            twist.linear.x = TRACE_SPEED;       twist.angular.z = TURN_MICRO;  debug_state="Left (L1)";
                        }
                        
                        // 右側の判定
                        else if (is_r4) { 
                            twist.linear.x = TRACE_SPEED * 0.3; twist.angular.z = -TURN_STRONG; debug_state="Right++ (R4)"; 
                        }
                        else if (is_r3) { 
                            twist.linear.x = TRACE_SPEED * 0.5; twist.angular.z = -TURN_MEDIUM; debug_state="Right (R3)"; 
                        }
                        else if (is_r2) {
                            twist.linear.x = TRACE_SPEED * 0.8; twist.angular.z = -TURN_SLIGHT; debug_state="Right (R2)";
                        }
                        else if (is_r1) {
                            twist.linear.x = TRACE_SPEED;       twist.angular.z = -TURN_MICRO;  debug_state="Right (R1)";
                        }
                        
                        // 直進
                        else { 
                            twist.linear.x = TRACE_SPEED; twist.angular.z = 0.0; debug_state="Straight"; // センサの狭間なら直進
                        }
                    }    
                }
                break;

            case RobotState::CHASING:
                if (ball_command_ == 'N') {
                    if (last_known_dist_ <= (TARGET_DISTANCE + 40.0)) {
                        RCLCPP_INFO(this->get_logger(), "Assuming CAUGHT (Last: %.1fcm).", last_known_dist_);
                        twist.linear.x = 0.0; twist.angular.z = 0.0;
                        current_state_ = RobotState::COLLECTING;
                        state_start_time_ = now;
                        
                        if (last_known_command_ == 'R') red_count_++;
                        else if (last_known_command_ == 'B') blue_count_++;
                        else if (last_known_command_ == 'Y') yellow_count_++;
                        RCLCPP_INFO(this->get_logger(), "Caught Ball! Total: %d", total_balls + 1);
                    } else {
                        current_state_ = RobotState::RETURNING;
                        state_start_time_ = now;
                        RCLCPP_INFO(this->get_logger(), "Lost Ball. Returning...");
                    }
                } 
                else {
                    double error = 320.0 - ball_center_x_;
                    twist.angular.z = 0.005 * error;
                    if (std::abs(error) < 50.0) {
                        if (ball_dist_ > TARGET_DISTANCE) {
                            twist.linear.x = 0.2; 
                            debug_state = "Chasing";
                        } else {
                            twist.linear.x = 0.0; twist.angular.z = 0.0;
                            current_state_ = RobotState::COLLECTING;
                            state_start_time_ = now;
                            if (ball_command_ == 'R') red_count_++;
                            else if (ball_command_ == 'B') blue_count_++;
                            else if (ball_command_ == 'Y') yellow_count_++;
                            RCLCPP_INFO(this->get_logger(), "Caught Ball! Total: %d", total_balls + 1);
                        }
                    } else {
                        twist.linear.x = 0.05; 
                        debug_state = "Aiming";
                    }
                }
                break;

            case RobotState::COLLECTING:
                twist.linear.x = 0.0; twist.angular.z = 0.0;
                if ((now - state_start_time_).seconds() > 2.0) {
                    current_state_ = RobotState::RETURNING;
                    state_start_time_ = now;
                    RCLCPP_INFO(this->get_logger(), "Start Returning...");
                }
                break;

            case RobotState::RETURNING:
                twist.linear.x = BACK_SPEED; twist.angular.z = 0.0;
                if (is_any_black) {
                    twist.linear.x = 0.0;
                    if (total_balls >= GOAL_BALL_COUNT) {
                        current_state_ = RobotState::U_TURN;
                        state_start_time_ = now; 
                        RCLCPP_INFO(this->get_logger(), "Goal Met! Starting U-Turn.");
                    } else {
                        current_state_ = RobotState::LINE_TRACING;
                        RCLCPP_INFO(this->get_logger(), "Back on Line. Next Ball.");
                    }
                }
                else if ((now - state_start_time_).seconds() > 5.0) {
                    current_state_ = RobotState::LINE_TRACING;
                    RCLCPP_INFO(this->get_logger(), "Time Out. Force Resume.");
                }
                break;
            
            case RobotState::U_TURN:
                twist.linear.x = 0.0; twist.angular.z = 1.0; 
                if ((now - state_start_time_).seconds() > 1.5) {
                    if (is_l1 || is_r1) { 
                        twist.angular.z = 0.0;
                        current_state_ = RobotState::GO_HOME;
                        cross_line_count_ = 0; 
                        RCLCPP_INFO(this->get_logger(), "U-Turn Complete! Going Home.");
                    }
                }
                break;

            case RobotState::GO_HOME:
                {
                    bool detected_cross_line = (is_l3 && is_r3);
                    if (detected_cross_line && !is_on_cross_line_) {
                        cross_line_count_++; is_on_cross_line_ = true;
                    } else if (!detected_cross_line) {
                        is_on_cross_line_ = false;
                    }
                    
                    if (is_on_cross_line_) {
                        twist.linear.x = TRACE_SPEED;
                        twist.angular.z = 0.0;
                    }
                    else{
                        // 帰りのライントレース (段階制御)
                        if (is_l4 && cross_line_count_ != 2) { 
                            twist.linear.x = TRACE_SPEED*0.3; twist.angular.z = TURN_STRONG; 
                        }
                        else if (is_l3 && cross_line_count_ != 2) { 
                            twist.linear.x = TRACE_SPEED*0.5; twist.angular.z = TURN_MEDIUM; 
                        }
                        else if (is_l2 && cross_line_count_ != 2) {
                            twist.linear.x = TRACE_SPEED*0.8; twist.angular.z = TURN_SLIGHT;
                        }
                        else if (is_l1 && cross_line_count_ != 2) {
                            twist.linear.x = TRACE_SPEED;     twist.angular.z = TURN_MICRO;
                        }
                        
                        else if (is_r4) { twist.linear.x = TRACE_SPEED*0.3; twist.angular.z = -TURN_STRONG; }
                        else if (is_r3) { twist.linear.x = TRACE_SPEED*0.5; twist.angular.z = -TURN_MEDIUM; }
                        else if (is_r2) { twist.linear.x = TRACE_SPEED*0.8; twist.angular.z = -TURN_SLIGHT; }
                        else if (is_r1) { twist.linear.x = TRACE_SPEED;     twist.angular.z = -TURN_MICRO; }
                        
                        else { twist.linear.x = TRACE_SPEED; twist.angular.z = 0.0; } 
                    }
                    if (cross_line_count_ > 10) current_state_ = RobotState::FINISHED;
                }
                debug_state = "Going Home";
                break;

            case RobotState::FINISHED:
                twist.linear.x = 0.0; twist.angular.z = 0.0;
                debug_state = "ALL FINISHED";
                break;
        }

        auto noisy_twist = apply_motor_noise(twist);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
            "[%s] Ball:%d | LineCnt:%d | Out: Lin:%.2f Ang:%.2f (Noisy)", 
            debug_state.c_str(), total_balls, cross_line_count_,
            twist.linear.x, twist.angular.z);

        publisher_->publish(noisy_twist);
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_ball_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_l4_, sub_l3_, sub_l2_, sub_l1_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_r1_, sub_r2_, sub_r3_, sub_r4_;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    RobotState current_state_;
    rclcpp::Time state_start_time_;
    rclcpp::Time chase_start_time_;
    
    uint8_t val_l4_=255, val_l3_=255, val_l2_=255, val_l1_=255;
    uint8_t val_r1_=255, val_r2_=255, val_r3_=255, val_r4_=255;

    char ball_command_ = 'N';
    double ball_dist_ = 0.0;
    double ball_center_x_ = 320.0;
    
    double last_known_dist_ = 999.0;
    char last_known_command_ = 'N';

    int cross_line_count_ = 0;
    bool is_on_cross_line_ = false;

    int red_count_ = 0;
    int blue_count_ = 0;
    int yellow_count_ = 0;

    std::mt19937 generator_;
    std::uniform_real_distribution<double> distribution_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BallChaser>());
    rclcpp::shutdown();
    return 0;
}