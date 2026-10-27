#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class LineDetectorNode : public rclcpp::Node {
public:
    explicit LineDetectorNode() : Node("line_detector_node") {
        sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&LineDetectorNode::image_callback, this, std::placeholders::_1)
        );

        pub_steer_target_ = this->create_publisher<std_msgs::msg::Float32>("/vision/steer_target", 10);
        RCLCPP_INFO(this->get_logger(), "Vision Line Detector Aktif.");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_steer_target_;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

            cv::Mat gray, binary;
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
            cv::threshold(gray, binary, 200, 255, cv::THRESH_BINARY);

            cv::Moments m = cv::moments(binary, true);
            float steer_err = 0.0f;

            if (m.m00 > 0) {
                float cx = m.m10 / m.m00;
                float img_center = frame.cols / 2.0f;
                steer_err = ((cx - img_center) / img_center) * 45.0f;
            }

            auto out_msg = std_msgs::msg::Float32();
            out_msg.data = steer_err;
            pub_steer_target_->publish(out_msg);

        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Vision Exception: %s", e.what());
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineDetectorNode>());
    rclcpp::shutdown();
    return 0;
}