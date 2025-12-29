#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <vector>
#include <sstream>
#include <cmath> 

enum class RobotState {
    LINE_TRACING,   // ライントレース
    CHASING,        // ボール追跡
    COLLECTING,     // 回収中
    RETURNING,      // バックでラインに戻る
    U_TURN,         // 180度回転
    GO_HOME,        // ゴールへ帰還
    FINISHED        // 終了
};

class BallChaser : public rclcpp::Node {
public:
    BallChaser() : Node("ball_chaser") {
        sub_ball_ = this->create_subscription<std_msgs::msg::String>(
            "/ball_info", 10, std::bind(&BallChaser::ball_callback, this, std::placeholders::_1));
        
        // 5つのセンサー
        sub_ll_ = this->create_subscription<sensor_msgs::msg::Image>("/line/left_left", 10, [this](const sensor_msgs::msg::Image::SharedPtr msg){ val_ll_ = msg->data[0]; });
        sub_l_  = this->create_subscription<sensor_msgs::msg::Image>("/line/left", 10,      [this](const sensor_msgs::msg::Image::SharedPtr msg){ val_l_  = msg->data[0]; });
        sub_c_  = this->create_subscription<sensor_msgs::msg::Image>("/line/center", 10,    [this](const sensor_msgs::msg::Image::SharedPtr msg){ val_c_  = msg->data[0]; });
        sub_r_  = this->create_subscription<sensor_msgs::msg::Image>("/line/right", 10,     [this](const sensor_msgs::msg::Image::SharedPtr msg){ val_r_  = msg->data[0]; });
        sub_rr_ = this->create_subscription<sensor_msgs::msg::Image>("/line/right_right", 10,[this](const sensor_msgs::msg::Image::SharedPtr msg){ val_rr_ = msg->data[0]; });

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        current_state_ = RobotState::LINE_TRACING;
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), std::bind(&BallChaser::control_loop, this));
            
        RCLCPP_INFO(this->get_logger(), "Ball Chaser (Smart Capture Logic) Started.");
    }

private:
    void ball_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::string data = msg->data;
        char cmd_buf[10]; 
        if (sscanf(data.c_str(), "%[^:]:%lf:%lf", cmd_buf, &ball_dist_, &ball_center_x_) >= 1) {
            ball_command_ = cmd_buf[0];
            
            // ★追加: ボールが見えている間、前回の距離と色を記憶しておく
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
        
        // センサー判定
        bool is_ll = (val_ll_ < BLACK_THRESHOLD);
        bool is_l  = (val_l_  < BLACK_THRESHOLD);
        bool is_c  = (val_c_  < BLACK_THRESHOLD);
        bool is_r  = (val_r_  < BLACK_THRESHOLD);
        bool is_rr = (val_rr_ < BLACK_THRESHOLD);
        bool is_any_black = (is_ll || is_l || is_c || is_r || is_rr);

        int total_balls = red_count_ + blue_count_ + yellow_count_;
        const int GOAL_BALL_COUNT = 1; 

        switch (current_state_) {
            // ==========================================
            // 0. ライントレース
            // ==========================================
            case RobotState::LINE_TRACING:
                // ※GO_HOME遷移はRETURNINGでの判定に任せるが、ここでも念のため
                if (total_balls >= GOAL_BALL_COUNT) {
                    // ここに来ることは稀（バック中にUターンするから）
                }

                if (ball_command_ != 'N' && ball_dist_ <= MAX_CHASE_DISTANCE) {
                    current_state_ = RobotState::CHASING;
                    chase_start_time_ = now;
                    // 追跡開始時に記憶変数をリセットしない（前の値を誤用しないよう注意）
                    last_known_dist_ = ball_dist_; 
                    RCLCPP_INFO(this->get_logger(), "Ball Found! Switch to CHASING.");
                    return;
                }

                // --- 通常トレース & 横線無視ロジック ---
                {
                    bool detected_cross_line = (is_ll && is_rr);

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
                        if (is_ll) { twist.linear.x = TRACE_SPEED * 0.3; twist.angular.z = TURN_SPEED_STRONG; debug_state="Left++"; }
                        else if (is_rr) { twist.linear.x = TRACE_SPEED * 0.3; twist.angular.z = -TURN_SPEED_STRONG; debug_state="Right++"; }
                        else if (is_l) { twist.linear.x = TRACE_SPEED * 0.5; twist.angular.z = TURN_SPEED_WEAK; debug_state="Left"; }
                        else if (is_r) { twist.linear.x = TRACE_SPEED * 0.5; twist.angular.z = -TURN_SPEED_WEAK; debug_state="Right"; }
                        else if (is_c) { twist.linear.x = TRACE_SPEED; twist.angular.z = 0.0; debug_state="Straight"; }
                        else { twist.linear.x = 0.0; twist.angular.z = 0.0; debug_state="Lost"; }
                    }
                }
                break;

            // ==========================================
            // 1. ボール追跡
            // ==========================================
            case RobotState::CHASING:
                if (ball_command_ == 'N') {
                    // ★重要修正: 見失ったとき、直前の距離が近ければ「確保成功」とみなす
                    // TARGET_DISTANCE(40cm) より少し余裕を持たせて +10cm 以内なら成功とする
                    if (last_known_dist_ <= (TARGET_DISTANCE + 40.0)) {
                        RCLCPP_INFO(this->get_logger(), "Lost ball but close (%.1fcm). Assuming CAUGHT.", last_known_dist_);
                        
                        twist.linear.x = 0.0;
                        twist.angular.z = 0.0;
                        current_state_ = RobotState::COLLECTING;
                        state_start_time_ = now;
                        
                        // 直前の色情報を使ってカウント
                        if (last_known_command_ == 'R') red_count_++;
                        else if (last_known_command_ == 'B') blue_count_++;
                        else if (last_known_command_ == 'Y') yellow_count_++;
                        
                        RCLCPP_INFO(this->get_logger(), "Caught Ball! Total: %d", total_balls + 1);

                    } else {
                        // 本当に見失った（遠い）
                        current_state_ = RobotState::RETURNING;
                        state_start_time_ = now;
                        RCLCPP_INFO(this->get_logger(), "Lost Ball (Too far: %.1fcm). Returning...", last_known_dist_);
                    }
                } 
                else {
                    // ボールが見えている場合
                    double error = 320.0 - ball_center_x_;
                    twist.angular.z = 0.005 * error;
                    
                    if (std::abs(error) < 50.0) {
                        if (ball_dist_ > TARGET_DISTANCE) {
                            twist.linear.x = 0.2; 
                            debug_state = "Chasing";
                        } else {
                            // 距離条件を満たして停止
                            twist.linear.x = 0.0;
                            twist.angular.z = 0.0;
                            current_state_ = RobotState::COLLECTING;
                            state_start_time_ = now;
                            
                            if (ball_command_ == 'R') red_count_++;
                            else if (ball_command_ == 'B') blue_count_++;
                            else if (ball_command_ == 'Y') yellow_count_++;
                            
                            RCLCPP_INFO(this->get_logger(), "Caught Ball (Dist OK)! Total: %d", total_balls + 1);
                        }
                    } else {
                        twist.linear.x = 0.05; 
                        debug_state = "Aiming";
                    }
                }
                break;

            // ==========================================
            // 2. 回収 (停止)
            // ==========================================
            case RobotState::COLLECTING:
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;
                debug_state = "Collecting...";
                
                if ((now - state_start_time_).seconds() > 2.0) {
                    current_state_ = RobotState::RETURNING;
                    state_start_time_ = now;
                    RCLCPP_INFO(this->get_logger(), "Start Returning...");
                }
                break;

            // ==========================================
            // 3. 復帰 (バック走行)
            // ==========================================
            case RobotState::RETURNING:
                twist.linear.x = BACK_SPEED; 
                twist.angular.z = 0.0;
                debug_state = "Returning";

                if (is_any_black) {
                    twist.linear.x = 0.0; // 停止

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
            
            // ==========================================
            // 4. Uターン (180度回転)
            // ==========================================
            case RobotState::U_TURN:
                twist.linear.x = 0.0;
                twist.angular.z = 1.0; 
                debug_state = "U-Turn";

                // 最初の1.5秒は無条件回転（ラインから外れるため）
                if ((now - state_start_time_).seconds() > 1.5) {
                    // その後、中央センサが黒になったら完了
                    if (is_c) {
                        twist.angular.z = 0.0;
                        current_state_ = RobotState::GO_HOME;
                        cross_line_count_ = 0; 
                        RCLCPP_INFO(this->get_logger(), "U-Turn Complete! Going Home.");
                    }
                }
                break;

            // ==========================================
            // 5. 帰還 (ゴールへ)
            // ==========================================
            case RobotState::GO_HOME:
                {
                    bool detected_cross_line = (is_ll && is_rr);
                    if (detected_cross_line && !is_on_cross_line_) {
                        cross_line_count_++;
                        is_on_cross_line_ = true;
                    } else if (!detected_cross_line) {
                        is_on_cross_line_ = false;
                    }

                    if (is_c) { twist.linear.x = TRACE_SPEED; twist.angular.z = 0.0; }
                    else if (is_l) { twist.linear.x = TRACE_SPEED*0.5; twist.angular.z = TURN_SPEED_WEAK; }
                    else if (is_r) { twist.linear.x = TRACE_SPEED*0.5; twist.angular.z = -TURN_SPEED_WEAK; }
                    else { twist.linear.x = 0.0; twist.angular.z = 0.0; } 

                    if (cross_line_count_ > 10) { 
                         current_state_ = RobotState::FINISHED;
                    }
                }
                debug_state = "Going Home";
                break;

            case RobotState::FINISHED:
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;
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
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_ll_, sub_l_, sub_c_, sub_r_, sub_rr_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    RobotState current_state_;
    rclcpp::Time state_start_time_;
    rclcpp::Time chase_start_time_;
    
    uint8_t val_ll_ = 255, val_l_ = 255, val_c_ = 255, val_r_ = 255, val_rr_ = 255;
    char ball_command_ = 'N';
    double ball_dist_ = 0.0;
    double ball_center_x_ = 320.0;
    
    // ★追加: 直前のボール情報を記憶する変数
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