#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <arpa/inet.h>
#include <endian.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <cstdint>
#include <cstring>
#include <thread>

struct VehicleFeedback {
    int32_t left_ticks{0};
    int32_t right_ticks{0};
    int32_t vel_x_cms{0};
    int32_t vel_y_cms{0};
};

class UdpReceiverNode : public rclcpp::Node {
public:
    explicit UdpReceiverNode() : Node("udp_receiver_node") {
        this->declare_parameter<int>("port", 12152);
        int port = this->get_parameter("port").as_int();

        if (!init_socket(port)) {
            RCLCPP_ERROR(this->get_logger(), "Gagal bind UDP Receiver Socket!");
            return;
        }

        pub_feedback_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/vehicle_feedback", 10);
        recv_thread_ = std::thread(&UdpReceiverNode::receive_loop, this);

        RCLCPP_INFO(this->get_logger(), "UDP Receiver mendengarkan pada port %d", port);
    }

    ~UdpReceiverNode() override {
        running_ = false;
        if (recv_thread_.joinable()) recv_thread_.join();
        if (sock_ >= 0) close(sock_);
    }

private:
    static constexpr char kHeader[4] = {'I', 'R', 'I', 'S'};
    static constexpr size_t kPacketSize = 20;

    int sock_{-1};
    bool running_{true};
    std::thread recv_thread_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_feedback_;

    bool init_socket(int port) {
        sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_ < 0) return false;

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(static_cast<uint16_t>(port));
        addr.sin_addr.s_addr = htonl(INADDR_ANY);

        if (bind(sock_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) return false;

        timeval tv{1, 0};
        setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        return true;
    }

    int32_t read_i32_le(const uint8_t *src) {
        uint32_t raw;
        std::memcpy(&raw, src, sizeof(raw));
        return static_cast<int32_t>(le32toh(raw));
    }

    bool parse_packet(const uint8_t *buf, size_t len, VehicleFeedback &out) {
        if (len < kPacketSize || std::memcmp(buf, kHeader, sizeof(kHeader)) != 0) return false;
        out.left_ticks = read_i32_le(buf + 4);
        out.right_ticks = read_i32_le(buf + 8);
        out.vel_x_cms = read_i32_le(buf + 12);
        out.vel_y_cms = read_i32_le(buf + 16);
        return true;
    }

    void receive_loop() {
        uint8_t buf[64];
        while (running_ && rclcpp::ok()) {
            ssize_t n = recvfrom(sock_, buf, sizeof(buf), 0, nullptr, nullptr);
            if (n > 0) {
                VehicleFeedback fb;
                if (parse_packet(buf, static_cast<size_t>(n), fb)) {
                    auto msg = geometry_msgs::msg::TwistStamped();
                    msg.header.stamp = this->now();
                    msg.twist.linear.x = fb.vel_x_cms;
                    msg.twist.linear.y = fb.vel_y_cms;
                    msg.twist.angular.x = fb.left_ticks;
                    msg.twist.angular.y = fb.right_ticks;
                    pub_feedback_->publish(msg);
                }
            }
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UdpReceiverNode>());
    rclcpp::shutdown();
    return 0;
}