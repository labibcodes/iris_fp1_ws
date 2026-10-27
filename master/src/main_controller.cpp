#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/twist.hpp>

class MainControllerNode : public rclcpp::Node {
public:
    explicit MainControllerNode() : Node("main_controller_node") {
        sub_steer_ = this->create_subscription<std_msgs::msg::Float32>(
            "/vision/steer_target", 10,
            std::bind(&MainControllerNode::steer_callback, this, std::placeholders::_1)
        );

        pub_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_raw", 10);
        RCLCPP_INFO(this->get_logger(), "Master Controller Aktif. Mobil Siap Jalan!");
    }

private:
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_steer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;

    const int32_t base_pwm_{100};

    void steer_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        auto cmd = geometry_msgs::msg::Twist();
        cmd.linear.x = base_pwm_;
        cmd.angular.z = msg->data;
        pub_cmd_->publish(cmd);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MainControllerNode>());
    rclcpp::shutdown();
    return 0;
}