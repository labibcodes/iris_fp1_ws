#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <arpa/inet.h>
#include <endian.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cstdint>
#include <cstring>

class UdpSenderNode : public rclcpp::Node {
public:
    explicit UdpSenderNode() : Node("udp_sender_node") {
        this->declare_parameter<std::string>("host", "127.0.0.1");
        this->declare_parameter<int>("port", 12151);

        std::string host = this->get_parameter("host").as_string();
        int port = this->get_parameter("port").as_int();

        if (!init_socket(host, port)) {
            RCLCPP_ERROR(this->get_logger(), "Gagal membuat UDP Sender Socket!");
            return;
        }

        sub_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_raw", 10,
            std::bind(&UdpSenderNode::cmd_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "UDP Sender siap mengirim ke %s:%d", host.c_str(), port);
    }

    ~UdpSenderNode() override {
        send_packet(0, current_steer_);
        if (sock_ >= 0) close(sock_);
    }

private:
    static constexpr char kHeader[4] = {'I', 'R', 'I', 'S'};
    static constexpr size_t kPacketSize = 12;

    int sock_{-1};
    sockaddr_in addr_{};
    int32_t current_steer_{0};
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;

    bool init_socket(const std::string &host, int port) {
        sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_ < 0) return false;

        addr_.sin_family = AF_INET;
        addr_.sin_port = htons(static_cast<uint16_t>(port));
        return inet_pton(AF_INET, host.c_str(), &addr_.sin_addr) == 1;
    }

    void build_packet(uint8_t (&buf)[kPacketSize], int32_t pwm, int32_t steer) {
        pwm = std::clamp<int32_t>(pwm, -2000, 2000);
        steer = std::clamp<int32_t>(steer, -90, 90);

        uint32_t pwm_le = htole32(static_cast<uint32_t>(pwm));
        uint32_t steer_le = htole32(static_cast<uint32_t>(steer));

        std::memcpy(buf, kHeader, sizeof(kHeader));
        std::memcpy(buf + 4, &pwm_le, sizeof(pwm_le));
        std::memcpy(buf + 8, &steer_le, sizeof(steer_le));
    }

    void send_packet(int32_t pwm, int32_t steer) {
        uint8_t packet[kPacketSize];
        build_packet(packet, pwm, steer);
        sendto(sock_, packet, sizeof(packet), 0,
               reinterpret_cast<sockaddr *>(&addr_), sizeof(addr_));
    }

    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        int32_t pwm = static_cast<int32_t>(msg->linear.x);
        int32_t steer = static_cast<int32_t>(msg->angular.z);
        current_steer_ = steer;
        send_packet(pwm, steer);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UdpSenderNode>());
    rclcpp::shutdown();
    return 0;
}