#include "rclcpp/rclcpp.hpp"
#include <string>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <cmath>

class WallFollow : public rclcpp::Node {

public:
    WallFollow() : Node("wall_follow_node")
    {
        // Lidar scan subscriber
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidarscan_topic, 10, std::bind(&WallFollow::scan_callback, this, std::placeholders::_1));
        // Drive publisher
        drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            drive_topic, 10);
    }

private:
    // PID CONTROL PARAMS
    double kp = 0.5;
    double kd = 0.01;
    double ki = 0.1;

    double servo_offset = 0.0;
    double prev_error = 0.0;
    double error = 0.0;
    double integral = 0.0;

    double dist_deseada = 0.85; // Desired distance to the wall
    double dist_L = 0.4; // Constanste para el instante siguiente

    // Topics
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";


    /// ROS subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;

    /*
    * Función auxiliar simple para devolver la medición de rango correspondiente en un ángulo dado.
    * Argumentos:
    *     range_data: arreglo de rangos único del LiDAR
    *     angle: entre angle_min y angle_max del LiDAR
    * 
    * Retorna:
    *     range: medición de rango en metros en el ángulo dado
    */
    double get_range(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg, double angle)
    {

        // Angulo en grados a radianes
        double angle_rad = angle * M_PI / 180.0;

        // Verificamos que esté dentro del rango del lidar
        if (angle_rad < scan_msg->angle_min || angle_rad > scan_msg->angle_max) {
            RCLCPP_WARN(this->get_logger(), "Angle out of range");
            return std::numeric_limits<double>::quiet_NaN();
        }

        // Calculo del indice
        int index = static_cast<int>((angle_rad - scan_msg->angle_min) / scan_msg->angle_increment);

        // Verificamos que el indice esté dentro del rango
        if (index < 0 || index >= scan_msg->ranges.size()) {
            RCLCPP_WARN(this->get_logger(), "Index out of range");
            return std::numeric_limits<double>::quiet_NaN();
        }

        // Obtenemos la distancia
        double range = scan_msg->ranges[index];

        return range;
    }

    /*
    Calcula el error con respecto a la pared. Sigue la pared a la izquierda (yendo en sentido antihorario en el circuito de Levine).
    
    Argumentos:
        range_data: arreglo único de rangos del LiDAR
        dist: distancia deseada a la pared

    Retorna:
        error: error calculado
    */
    double get_error(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg, double dist_deseada)
    {
        double dist_a = get_range(scan_msg, 60.0);
        double dist_b = get_range(scan_msg, 90.0);
        double theta_rad = 30 * M_PI / 180.0;
        double alpha = atan2(dist_a*cos(theta_rad) - dist_b, dist_a*sin(theta_rad));

        double dist_AB = dist_b * cos(alpha);
        double dist_CD = dist_AB + dist_L * sin(alpha);

        double dist_error = dist_CD - dist_deseada;

        return dist_error;
    }


    /**
     * Función para controlar el vehículo utilizando un controlador PID.
     * Esta función calcula el valor de control basado en el error actual, el error previo
     * y los parámetros del controlador PID (kp, kd, ki). Luego publica un mensaje de
     * AckermannDriveStamped con el ángulo de dirección y la velocidad calculados.
     */
    void pid_control(double error, double velocity)
    {
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        this->error = error;

        double pid = kp * error + kd * (error - prev_error);
        double angle = 1 * pid;

        drive_msg.drive.steering_angle = angle;
        drive_msg.drive.speed = velocity;

        // Publicar el mensaje de conducción
        drive_publisher_->publish(drive_msg);

        // Imprimir calculo de pid para debug
        RCLCPP_INFO(this->get_logger(), "PID: %f", pid);

        // Guardar el error para la siguiente iteracion
        prev_error = error;
    }

    /**
     * Función de callback para los mensajes del escáner LIDAR.
     * Esta función procesa los datos entrantes del LIDAR para calcular el error
     * con respecto a la distancia deseada de la pared y luego utiliza un controlador PID
     * para determinar el ángulo de dirección y la velocidad apropiados para el vehículo.
     */
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        // Calcular el error
        double error = get_error(scan_msg, dist_deseada);
        double velocity = 1.50; // TODO: calculate desired car velocity based on error
        
        // Conducir el coche con PID
        pid_control(error, velocity);
    }

};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}