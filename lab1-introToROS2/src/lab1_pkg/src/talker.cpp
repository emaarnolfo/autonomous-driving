#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

class Talker : public rclcpp::Node{
public:
    Talker() : Node("talker"){
        this->declare_parameter("v", 1.0);
        this->declare_parameter("d", 0.0);

        speed_ = this->get_parameter("v").as_double();
        angle_ = this->get_parameter("d").as_double();

        //Crear el publicador
        publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);

        // Crear un temporizador para publicar periódicamente
        timer_ = this-> (std::chrono::seconds(1), std::bind(&Talker::publish_message, this));
    }

private:
    void publish_message()
    {
        auto msg = ackermann_msgs::msg::AckermannDriveStamped();
        msg.drive.speed = speed_;
        msg.drive.steering_angle = angle_;

        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Publicando: SPEED=%.2f, angle=%.2f", speed_, angle_);
    }

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double speed_;
    double angle_;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Talker>());
    rclcpp::shutdown();
    return 0;
}