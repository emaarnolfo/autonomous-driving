#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "cmath"
/// CHECK: include needed ROS msg type headers and libraries

class ReactiveFollowGap : public rclcpp::Node {
// Implement Reactive Follow Gap on the car
// This is just a template, you are free to implement your own node!

public:
    ReactiveFollowGap() : Node("reactive_node")
    {
        lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidarscan_topic, 10, std::bind(&ReactiveFollowGap::lidar_callback, this, std::placeholders::_1));
        /// TODO: create ROS subscribers and publishers
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            drive_topic, 10);
        RCLCPP_INFO(this->get_logger(), "Reactive Follow Gap Node has been started");
    }

private:
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    const int max_gap = 10; // max gap value
    const float bubble_radius = 0.5; // bubble radius in meters
    const float max_steering_angle = M_PI / 4; // max steering angle in radians 
    const float SMOOTHINH_FACTOR = 0.8; // smoothing factor for the bubble
    const int max_jump = 10;

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

    /**
     * Method to preprocess the LiDAR scan array. Expert implementation includes:
     * 1. Setting each value to the mean over a window of 5 values
     * 2. Rejecting high values
     */
    void preprocess_lidar(std::vector<float> &ranges)
    {   
        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over a window of 5 values
        for(int i = 0; i < ranges.size(); i++)
        {
            if(i < 2 || i > ranges.size() - 3)
                continue;

            // Set each value to the mean over a window of 5 values
            ranges[i] = (ranges[i-2] + ranges[i-1] + ranges[i] + ranges[i+1] + ranges[i+2]) / 5;
            
        }
        //2.Rejecting high values
        for(int i = 0; i < ranges.size(); i++)
        {
            if(ranges[i] > this->max_gap)
                ranges[i] = this->max_gap;
        }
        return;
    }

    void find_closest_point(std::vector<float>& ranges, int& indice)
    {   
        // Find the closest point in ranges
        for(int i = 0; i < ranges.size(); i++)
        {
            if(ranges[i] < ranges[indice])
                indice = i;
        }
        return;
    }

    void apply_bubble(std::vector<float>& ranges, int closest_index, float angle_min, float angle_increment, float bubble_radius)
    {   
        float r = ranges[closest_index];

        // Si el punta está demasiado cerca, burbuja maxima
        if(r < bubble_radius)
            r = bubble_radius;

        float theta = std::asin(bubble_radius / r);
        int n = static_cast<int>(theta / angle_increment);

        int start = std::max(0, closest_index - n);
        int end = std::min((int)ranges.size() - 1, closest_index + n);

        for (int i = start; i < end; i++)
        {
            ranges[i] = 0.0f;
        }
        
        return;
    }

    void find_max_gap(const std::vector<float>& ranges, int& start_i, int& end_i)
    {
        int max_length = 0;
        int current_start = -1;

        start_i = -1;
        end_i = -1;

        for (int i = 0; i < ranges.size(); ++i)
        {
            if (ranges[i] > 0)  // solo consideramos puntos válidos
            {
                if (current_start == -1)
                    current_start = i;

                // Si es el último punto y seguimos en un gap
                if (i == ranges.size() - 1 && current_start != -1)
                {
                    int length = i - current_start;
                    if (length > max_length)
                    {
                        max_length = length;
                        start_i = current_start;
                        end_i = i;
                    }
                }
            }
            else
            {
                if (current_start != -1)
                {
                    int length = i - current_start;
                    if (length > max_length)
                    {
                        max_length = length;
                        start_i = current_start;
                        end_i = i - 1;
                    }
                    current_start = -1; // reiniciar el inicio del gap
                }
            }
        }
        // Fallback si no se encontró ningún gap válido
        if (start_i == -1 || end_i == -1)
        {
            int mid = ranges.size() / 2;
            start_i = mid;
            end_i = mid;
        }
    }

    void find_best_point(float* ranges, int& start_i, int& end_i, int& best_index)
    {
        float max_range = 0.0;
        std::vector<int> best_indices;

        for (int i = start_i; i <= end_i; i++)
        {
            if (ranges[i] > max_range)
            {
                max_range = ranges[i];
                best_indices.clear();
                best_indices.push_back(i);
            }
            else if (ranges[i] == max_range)
            {
                best_indices.push_back(i);
            }
        }

        // Si no se encontró ningún índice válido, usar el índice medio
        static int previous_best_index = -1;

        if (!best_indices.empty())
        {
            int raw_index = best_indices[best_indices.size() / 2];

            // Filtro de suavizado
            if (previous_best_index == -1)
                previous_best_index = raw_index;
    
            // Aplica el filtro suave
            float alpha = 0.1;
            int smoothed_index = static_cast<int>(alpha * raw_index + (1.0 - alpha) * previous_best_index);
    
            // Aplica el límite de salto
            int delta = smoothed_index - previous_best_index;
            if (std::abs(delta) > max_jump)
            {
                if (delta > 0)
                    smoothed_index = previous_best_index + max_jump;
                else
                    smoothed_index = previous_best_index - max_jump;
            }

            best_index = smoothed_index;
            previous_best_index = smoothed_index;
            }

        previous_best_index = best_index;
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {   
        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        std::vector<float> ranges = scan_msg->ranges;
        static float previous_angle = 0.0; // previous angle for smoothing


        // 2. Preprocess the LiDAR scan
        preprocess_lidar(ranges);

        // 3. Find the closest point
        int closest_index = 0;
        find_closest_point(ranges, closest_index); 

        // 4. Apply bubble
        apply_bubble(ranges, closest_index, 
                     scan_msg->angle_min, 
                     scan_msg->angle_increment, bubble_radius);

        // 5. Find max length gap
        int gap_start = 0, gap_end = 0;
        find_max_gap(ranges, gap_start, gap_end);
        
        // 6. Find the best point in the gap
        int best_index = gap_start;
        find_best_point(ranges.data(), gap_start, gap_end, best_index);

        // 7. Calcular el ángulo global del mejor punto
        float angle = scan_msg->angle_min + best_index * scan_msg->angle_increment;

        float smooth_angle = SMOOTHINH_FACTOR * previous_angle + (1 - SMOOTHINH_FACTOR) * angle;
        previous_angle = smooth_angle;
        
        if(smooth_angle > max_steering_angle)
            smooth_angle = max_steering_angle;
        else if(smooth_angle < -max_steering_angle)
            smooth_angle = -max_steering_angle;

        // 8. Crear y publicar el mensaje de control
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.header.stamp = this->get_clock()->now();
        drive_msg.drive.steering_angle = smooth_angle;
        drive_msg.drive.speed = 3;  // podés ajustar la velocidad si querés

        RCLCPP_INFO(this->get_logger(), "Gap start: %d, end: %d", gap_start, gap_end);
        RCLCPP_INFO(this->get_logger(), "Best index: %d, angle: %.2f deg, distance: %.2f", best_index, angle * 180 / M_PI, ranges[best_index]);


        drive_pub_->publish(drive_msg);

    }
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;         //subscriber to scan msg
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;    //publisher to drive msg

};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}