#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

class MainControllerNode : public rclcpp::Node {
public:
    explicit MainControllerNode() : Node("main_controller_node") {
        // 1. Parameter Dinamis: Kecepatan Target (Default PWM 100) & Mode Kontrol
        this->declare_parameter<double>("target_speed", 100.0);
        this->declare_parameter<bool>("manual_mode", false);

        // 2. Subscriber Input Otonom dari Vision
        sub_steer_auto_ = this->create_subscription<std_msgs::msg::Float32>(
            "/vision/steer_target", 10,
            std::bind(&MainControllerNode::auto_steer_callback, this, std::placeholders::_1)
        );

        // 3. Subscriber Input Manual dari Teleop Keyboard
        sub_cmd_manual_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_manual", 10,
            std::bind(&MainControllerNode::manual_cmd_callback, this, std::placeholders::_1)
        );

        // 4. Subscriber Balikan Telemetri dari UDP Receiver (Tahu Posisi & Kecepatan Mobil)
        sub_feedback_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/vehicle_feedback", 10,
            std::bind(&MainControllerNode::feedback_callback, this, std::placeholders::_1)
        );

        // 5. Publisher Perintah Gerak ke UDP Sender
        pub_cmd_raw_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_raw", 10);

        RCLCPP_INFO(this->get_logger(), "=== MASTER CONTROLLER READY ===");
        RCLCPP_INFO(this->get_logger(), "Mode Bawaan: AUTONOMOUS. Gunakan 'ros2 param set /main_controller_node manual_mode true' untuk Mode Manual.");
    }

private:
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_steer_auto_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_manual_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_feedback_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_raw_;

    int32_t total_left_ticks_{0};
    int32_t total_right_ticks_{0};

    // Callback 1: Mode Otonom (Autonomous Line Following)
    void auto_steer_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        bool is_manual = this->get_parameter("manual_mode").as_bool();
        if (is_manual) return; // Jika mode manual aktif, abaikan vision

        double speed_pwm = this->get_parameter("target_speed").as_double();

        auto cmd = geometry_msgs::msg::Twist();
        cmd.linear.x = speed_pwm;      // Kecepatan PWM
        cmd.angular.z = msg->data;     // Sudut setir dari vision
        pub_cmd_raw_->publish(cmd);
    }

    // Callback 2: Mode Manual (Teleop Keyboard)
    void manual_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        bool is_manual = this->get_parameter("manual_mode").as_bool();
        if (!is_manual) return; // Jika mode otonom aktif, abaikan keyboard

        auto cmd = geometry_msgs::msg::Twist();
        cmd.linear.x = msg->linear.x;   // PWM dari keyboard teleop
        cmd.angular.z = -msg->angular.z; // Steer dari keyboard teleop
        pub_cmd_raw_->publish(cmd);
    }

    // Callback 3: Membaca Balikan Posisi & Kecepatan Mobil dari UDP Receiver
    void feedback_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        total_left_ticks_ = static_cast<int32_t>(msg->twist.angular.x);
        total_right_ticks_ = static_cast<int32_t>(msg->twist.angular.y);
        double vel_x = msg->twist.linear.x; // Kecepatan X (cm/s)
        double vel_y = msg->twist.linear.y; // Kecepatan Y (cm/s)

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
            "[TELEMETRI MOBIL] Kecepatan: X=%.1f cm/s, Y=%.1f cm/s | Odom Ticks: L=%d, R=%d",
            vel_x, vel_y, total_left_ticks_, total_right_ticks_);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MainControllerNode>());
    rclcpp::shutdown();
    return 0;
}