#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

class Relay : public rclcpp::Node {
public:
    Relay() : Node("relay") {
        // Suscribirse a "drive"
        subscription_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "drive", 10, std::bind(&Relay::listener_callback, this, std::placeholders::_1));

        // Publicar en "drive_relay"
        publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive_relay", 10);
    }

private:
    void listener_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
        // Modificar valores
        msg->drive.speed *= 3;
        msg->drive.steering_angle *= 3;

        // Publicar nuevo mensaje
        publisher_->publish(*msg);
        RCLCPP_INFO(this->get_logger(), "Publicando: SPEED=%.2f, angle=%.2f", msg->drive.speed, msg->drive.steering_angle);

    }

    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr subscription_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Relay>());
    rclcpp::shutdown();
    return 0;
}