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
        
        // ★8つのセンサを購読 (l4が一番左端, r4が一番右端)
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
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), std::bind(&BallChaser::control_loop, this));
            
        RCLCPP_INFO(this->get_logger(), "Ball Chaser (8 Sensors Stable Logic) Started.");
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

    void control_loop() {
        auto twist = geometry_msgs::msg::Twist();
        auto now = this->now();
        
        const double TARGET_DISTANCE = 40.0;
        const double MAX_CHASE_DISTANCE = 70.0;
        const int BLACK_THRESHOLD = 120;
        
        const double TRACE_SPEED = 0.15;
        const double TURN_SPEED_WEAK = 0.5; 
        const double TURN_SPEED_STRONG = 0.9;
        const double BACK_SPEED = -0.2; 

        std::string debug_state = "STOP";
        
        // 8つのセンサ判定
        bool is_l4 = (val_l4_ < BLACK_THRESHOLD); // 最左
        bool is_l3 = (val_l3_ < BLACK_THRESHOLD);
        bool is_l2 = (val_l2_ < BLACK_THRESHOLD);
        bool is_l1 = (val_l1_ < BLACK_THRESHOLD); // 内側左
        
        bool is_r1 = (val_r1_ < BLACK_THRESHOLD); // 内側右
        bool is_r2 = (val_r2_ < BLACK_THRESHOLD);
        bool is_r3 = (val_r3_ < BLACK_THRESHOLD);
        bool is_r4 = (val_r4_ < BLACK_THRESHOLD); // 最右

        // バック復帰用（どれか一つでも黒ならライン上）
        bool is_any_black = (is_l4 || is_l3 || is_l2 || is_l1 || is_r1 || is_r2 || is_r3 || is_r4);

        int total_balls = red_count_ + blue_count_ + yellow_count_;
        const int GOAL_BALL_COUNT = 1; 

        switch (current_state_) {
            // ==========================================
            // 0. ライントレース
            // ==========================================
            case RobotState::LINE_TRACING:
                if (total_balls >= GOAL_BALL_COUNT) {
                    // 通過
                }

                if (ball_command_ != 'N' && ball_dist_ <= MAX_CHASE_DISTANCE) {
                    current_state_ = RobotState::CHASING;
                    chase_start_time_ = now;
                    last_known_dist_ = ball_dist_; 
                    RCLCPP_INFO(this->get_logger(), "Ball Found! Switch to CHASING.");
                    return;
                }

                // --- 8センサ ライントレースロジック ---
                {
                    // 横線検知: 両端(L4, R4)が同時に黒なら横線とみなす
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
                        // ★修正: 外側(3,4)で曲がる、内側(1,2)は直進
                        // cross_line_count_ != 2 の左折禁止制限も維持

                        // --- 左旋回 (Outer Sensors) ---
                        if (is_l4 && cross_line_count_ != 2) { 
                            twist.linear.x = TRACE_SPEED * 0.3; 
                            twist.angular.z = TURN_SPEED_STRONG; 
                            debug_state="Left++ (L4)"; 
                        }
                        else if (is_l3 && cross_line_count_ != 2) { 
                            twist.linear.x = TRACE_SPEED * 0.5; 
                            twist.angular.z = TURN_SPEED_WEAK; 
                            debug_state="Left (L3)"; 
                        }
                        // --- 右旋回 (Outer Sensors) ---
                        else if (is_r4) { 
                            twist.linear.x = TRACE_SPEED * 0.3; 
                            twist.angular.z = -TURN_SPEED_STRONG; 
                            debug_state="Right++ (R4)"; 
                        }
                        else if (is_r3) { 
                            twist.linear.x = TRACE_SPEED * 0.5; 
                            twist.angular.z = -TURN_SPEED_WEAK; 
                            debug_state="Right (R3)"; 
                        }
                        // --- 直進 (Inner Sensors: L2, L1, R1, R2) ---
                        // 内側のセンサのどれかが黒なら直進する
                        else if (is_l2 || is_l1 || is_r1 || is_r2) { 
                            twist.linear.x = TRACE_SPEED; 
                            twist.angular.z = 0.0; 
                            debug_state="Straight (Inner)"; 
                        }
                        // --- ロスト ---
                        else { 
                            twist.linear.x = 0.0; 
                            twist.angular.z = 0.0; 
                            debug_state="Lost"; 
                        }
                    }    
                }
                break;

            // ==========================================
            // 1. ボール追跡
            // ==========================================
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
                    if (is_l1 || is_r1) { // 中央付近のセンサで検知したら完了
                        twist.angular.z = 0.0;
                        current_state_ = RobotState::GO_HOME;
                        cross_line_count_ = 0; 
                        RCLCPP_INFO(this->get_logger(), "U-Turn Complete! Going Home.");
                    }
                }
                break;

            case RobotState::GO_HOME:
                {
                    bool detected_cross_line = (is_l4 && is_r4);
                    if (detected_cross_line && !is_on_cross_line_) {
                        cross_line_count_++; is_on_cross_line_ = true;
                    } else if (!detected_cross_line) {
                        is_on_cross_line_ = false;
                    }

                    // 帰還時のライントレース (8センサ版)
                    if (is_l4) { twist.linear.x = TRACE_SPEED*0.5; twist.angular.z = TURN_SPEED_STRONG; }
                    else if (is_l3) { twist.linear.x = TRACE_SPEED*0.5; twist.angular.z = TURN_SPEED_WEAK; }
                    else if (is_r4) { twist.linear.x = TRACE_SPEED*0.5; twist.angular.z = -TURN_SPEED_STRONG; }
                    else if (is_r3) { twist.linear.x = TRACE_SPEED*0.5; twist.angular.z = -TURN_SPEED_WEAK; }
                    else if (is_l2 || is_l1 || is_r1 || is_r2) { twist.linear.x = TRACE_SPEED; twist.angular.z = 0.0; }
                    else { twist.linear.x = 0.0; twist.angular.z = 0.0; } 

                    if (cross_line_count_ > 10) current_state_ = RobotState::FINISHED;
                }
                debug_state = "Going Home";
                break;

            case RobotState::FINISHED:
                twist.linear.x = 0.0; twist.angular.z = 0.0;
                debug_state = "ALL FINISHED";
                break;
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
            "[%s] Ball:%d | LineCnt:%d | Out: Lin:%.2f Ang:%.2f", 
            debug_state.c_str(), total_balls, cross_line_count_,
            twist.linear.x, twist.angular.z);

        publisher_->publish(twist);
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_ball_;
    // 8つのセンサ
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_l4_, sub_l3_, sub_l2_, sub_l1_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_r1_, sub_r2_, sub_r3_, sub_r4_;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    RobotState current_state_;
    rclcpp::Time state_start_time_;
    rclcpp::Time chase_start_time_;
    
    // センサ値
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
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BallChaser>());
    rclcpp::shutdown();
    return 0;
}