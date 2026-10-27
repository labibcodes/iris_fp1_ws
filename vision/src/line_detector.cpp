#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class LineDetectorNode : public rclcpp::Node {
public:
    explicit LineDetectorNode() : Node("line_detector_node") {
        sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/catvehicle/camera_front/image_raw_front", 10,
            std::bind(&LineDetectorNode::image_callback, this, std::placeholders::_1)
        );

        pub_steer_target_ = this->create_publisher<std_msgs::msg::Float32>("/vision/steer_target", 10);
        RCLCPP_INFO(this->get_logger(), "Vision Line Detector (Robust Dual-Masking) Aktif.");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_steer_target_;
    float last_steer_err_ = 0.0f;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

            // Crop area jalan (Y=55% - 78%) -> Potong kap & langit
            int roi_y = frame.rows * 0.55;
            int roi_height = frame.rows * 0.23;
            cv::Rect roi(0, roi_y, frame.cols, roi_height);
            cv::Mat cropped_frame = frame(roi);

            // Canny Edge Detection untuk deteksi garis tajam
            cv::Mat gray, blur, edges;
            cv::cvtColor(cropped_frame, gray, cv::COLOR_BGR2GRAY);
            cv::GaussianBlur(gray, blur, cv::Size(5, 5), 0);
            cv::Canny(blur, edges, 50, 150);

            // Bagi ROI secara eksplisit jadi 2 Wilayah: KIRI (0-50%) & KANAN (50-100%)
            int mid_x = edges.cols / 2;
            cv::Mat left_mask = cv::Mat::zeros(edges.size(), CV_8UC1);
            cv::Mat right_mask = cv::Mat::zeros(edges.size(), CV_8UC1);

            edges(cv::Rect(0, 0, mid_x, edges.rows)).copyTo(left_mask(cv::Rect(0, 0, mid_x, edges.rows)));
            edges(cv::Rect(mid_x, 0, edges.cols - mid_x, edges.rows)).copyTo(right_mask(cv::Rect(mid_x, 0, edges.cols - mid_x, edges.rows)));

            cv::Moments m_left = cv::moments(left_mask, true);
            cv::Moments m_right = cv::moments(right_mask, true);

            bool left_detected = (m_left.m00 > 15);
            bool right_detected = (m_right.m00 > 15);

            float img_center = cropped_frame.cols / 2.0f;
            float target_cx = img_center;

            if (left_detected && right_detected) {
                // Kasus 1: Kedua garis terdeteksi -> Target di tengah-tengah
                float left_cx = m_left.m10 / m_left.m00;
                float right_cx = m_right.m10 / m_right.m00;
                target_cx = (left_cx + right_cx) / 2.0f;

                cv::circle(cropped_frame, cv::Point(left_cx, cropped_frame.rows / 2), 4, cv::Scalar(255, 0, 0), -1);
                cv::circle(cropped_frame, cv::Point(right_cx, cropped_frame.rows / 2), 4, cv::Scalar(0, 255, 0), -1);
            } 
            else if (right_detected) {
                // Kasus 2: Hanya garis kanan -> Target di sebelah KIRI garis kanan sejauh 65px
                float right_cx = m_right.m10 / m_right.m00;
                target_cx = right_cx - 65.0f;
                cv::circle(cropped_frame, cv::Point(right_cx, cropped_frame.rows / 2), 4, cv::Scalar(0, 255, 0), -1);
            } 
            else if (left_detected) {
                // Kasus 3: Hanya garis kiri -> Target di sebelah KANAN garis kiri sejauh 65px
                float left_cx = m_left.m10 / m_left.m00;
                target_cx = left_cx + 65.0f;
                cv::circle(cropped_frame, cv::Point(left_cx, cropped_frame.rows / 2), 4, cv::Scalar(255, 0, 0), -1);
            }

            float steer_err = 0.0f;
            if (left_detected || right_detected) {
                // Hitung error kemudi dengan gain 60.0f
                steer_err = ((target_cx - img_center) / img_center) * 60.0f;
                last_steer_err_ = steer_err;
                
                // Titik target merah (Tengah Lajur)
                cv::circle(cropped_frame, cv::Point(target_cx, cropped_frame.rows / 2), 6, cv::Scalar(0, 0, 255), -1);
            } else {
                // Jika garis hilang sementara di tikungan, gunakan belokan terakhir
                steer_err = last_steer_err_ * 0.90f;
            }

            cv::imshow("Edges Line", edges);
            cv::imshow("Camera Line", cropped_frame);
            cv::waitKey(1);

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