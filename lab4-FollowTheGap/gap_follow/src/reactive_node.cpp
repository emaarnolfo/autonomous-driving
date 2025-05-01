#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "cmath"
#include <fstream>
#include <vector>
#include <algorithm>

/// CHECK: include needed ROS msg type headers and libraries

std::pair<int, int> find_truncated_indices(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg, float truncation_angle_coverage)
{
    int start_index = 0;
    int end_index = 0;
    
    int truncated_range_size = static_cast<int>(truncation_angle_coverage / scan_msg->angle_increment);
    start_index = (scan_msg->ranges.size()/2) - (truncated_range_size/2);
    end_index = (scan_msg->ranges.size()/2) + (truncated_range_size/2);
    
    return std::make_pair(start_index, end_index);
}


void guardar_en_archivo(const std::vector<float>& datos, const std::string& nombre_archivo)
{
    std::ofstream archivo(nombre_archivo, std::ios::app); // modo append (no sobreescribe)
    if (!archivo.is_open())
    {
        //ROS_ERROR("No se pudo abrir el archivo para escribir.");
        RCLCPP_ERROR(rclcpp::get_logger("guardar_en_archivo"), "No se pudo abrir el archivo para escribir.");
        return;
    }

    for (const auto& valor : datos)
    {
        archivo << valor << " ";
    }
    archivo << std::endl; // salto de línea después del array

    archivo.close();
}


class ReactiveFollowGap : public rclcpp::Node {
public:
    ReactiveFollowGap() 
        : Node("reactive_node"),
          truncated(false),
          error_based_velocities_({
              {"low", 3.0},
              {"medium", 1.5},
              {"high", 0.5}
          })
    {
        lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidarscan_topic, 10, std::bind(&ReactiveFollowGap::lidar_callback, this, std::placeholders::_1));
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            drive_topic, 10);
        RCLCPP_INFO(this->get_logger(), "Reactive Follow Gap Node has been started");
    }

private:
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";

    const float SMOOTHING_FACTOR = 0.6; // smoothing factor for the bubble
    const int max_jump = 10;
    
    const int smoothing_filter_size = 5; // number of points to consider for smoothing
    const float max_gap = 10.0; // maximum gap size in meters
    const float bubble_radius = 0.5; // bubble radius in meters
    const float truncation_angle_coverage = M_PI; // max steering angle in radians 
    std::map<std::string, double> error_based_velocities_;
    
    
    int truncated_start_index;
    int truncated_end_index;
    bool truncated = false;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;         //subscriber to scan msg
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;    //publisher to drive msg

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
    std::vector<float> preprocess_lidar(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
    {   
        std::vector<float> ranges_filtered;
        for(int i = truncated_start_index; i < truncated_end_index; i++)
        {
            float sum = 0.0;
            for(int j = -smoothing_filter_size; j <= smoothing_filter_size; j++)
            {
                sum += scan_msg->ranges[i + j];
            }
            float mean = sum / (2 * smoothing_filter_size + 1);
            ranges_filtered.push_back(mean);
        }

        return ranges_filtered;
    }

    int find_closest_point(std::vector<float>& ranges)
    {   
        int index = 0;
        // Find the closest point in ranges
        for(int i = 0; i < ranges.size(); i++)
        {
            if(ranges[i] < ranges[index])
                index = i;
        }
        return index;
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
            if (ranges[i] > 0.1)  // solo consideramos puntos válidos
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

    void find_best_point(const std::vector<float>& ranges, int& start_i, int& end_i, int& best_index)
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

    //Alternativa 
    int get_best_point(const std::vector<float>& ranges, int start_i, int end_i)
    {
        return (start_i + end_i) / 2;
    }

    double get_steering_angle_from_range_index(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg,
            const size_t best_point_index,
            const double closest_value)
    {
        const size_t best_point_index_input_scan_frame = truncated_start_index + best_point_index;
        double best_point_steering_angle;

        // Case When Steering Angle is to be negative
        if(best_point_index_input_scan_frame < scan_msg->ranges.size()/2)
        {
            best_point_steering_angle = - scan_msg->angle_increment*
            static_cast<double>(scan_msg->ranges.size()/2.0 - best_point_index_input_scan_frame);
        }
        // Case When Steering Angle is to be negative
        else
        {
            best_point_steering_angle = scan_msg->angle_increment*
            static_cast<double>(best_point_index_input_scan_frame - scan_msg->ranges.size()/2.0);
        }
        const auto distance_compensated_steering_angle =
        std::clamp((best_point_steering_angle*SMOOTHING_FACTOR)/
        static_cast<double>(closest_value), -1.57, 1.57);
        return distance_compensated_steering_angle;
    }

    void zero_out_safety_bubble(std::vector<float>* input_vector, const size_t center_index,
            const double bubble_indices)    {
        const double center_point_distance =  input_vector->at(center_index);
        input_vector->at(center_index) = 0.0;

        // size_t current_index = center_index;
        // while((current_index < input_vector->size()-1) &&
        //         (input_vector->at(++current_index) < (center_point_distance + bubble_indices)))
        // {
        //     input_vector->at(current_index) = 0.0;
        // }

        // current_index = center_index;
        // while(current_index > 0 &&
        //         (input_vector->at(--current_index)) < (center_point_distance + bubble_indices))
        // {
        //     input_vector->at(current_index) = 0.0;
        // }
        for (size_t i = center_index - bubble_indices; i < center_index + bubble_indices; i++)
        {
            if(i < 0 || i >= input_vector->size())
                continue;
            input_vector->at(i) = 0.0;
        }
        
    }


    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {   
        static float previous_angle = 0.0; // previous angle for smoothing

        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        
        if(!truncated)
        {
            //RCLCPP_INFO(this->get_logger(), "Received LiDAR scan with %d points", (int)ranges.size());
            auto truncated_indices = find_truncated_indices(scan_msg, truncation_angle_coverage);
            truncated_start_index = truncated_indices.first;
            truncated_end_index = truncated_indices.second;
            truncated = true;
            RCLCPP_INFO(this->get_logger(), "Truncated indices: %d, %d", truncated_start_index, truncated_end_index);
        }
        
        // 1. Preprocess the LiDAR scan
        std::vector<float> filtered_ranges = preprocess_lidar(scan_msg);

        // 3. Find the closest point
        int closest_index = find_closest_point(filtered_ranges); 
        auto closest_range = filtered_ranges[closest_index];
        //RCLCPP_INFO(this->get_logger(), "Closest index: %d, distance: %.2f", closest_index, filtered_ranges[closest_index]);

        // 4. Apply bubble
        apply_bubble(filtered_ranges, closest_index, 
                    scan_msg->angle_min, 
                    scan_msg->angle_increment, bubble_radius);

        // zero_out_safety_bubble(&filtered_ranges, closest_index, bubble_indices);

        // 5. Find max length gap
        int gap_start = 0, gap_end = 0;
        find_max_gap(filtered_ranges, gap_start, gap_end);
        
        // 6. Find the best point in the gap
        int best_index = gap_start;
        find_best_point(filtered_ranges, gap_start, gap_end, best_index);

        // Obtiene el punto medio del gap
        //int best_index = get_best_point(filtered_ranges, gap_start, gap_end);

        // 7. Calcular el ángulo global del mejor punto
        //float steering_angle = scan_msg->angle_min + (best_index + truncated_start_index) * scan_msg->angle_increment;
        const double steering_angle = get_steering_angle_from_range_index(scan_msg, best_index, closest_range);
        RCLCPP_INFO(this->get_logger(), "Steering angle: %.f deg", steering_angle * 180 / M_PI);

        // Publish Drive message
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.header.stamp = this->get_clock()->now();
        drive_msg.header.frame_id = "laser";
        drive_msg.drive.steering_angle = steering_angle;
        if(abs(steering_angle) > 0.349)
        {
            drive_msg.drive.speed = error_based_velocities_["high"];
        }
        else if(abs(steering_angle) > 0.174)
        {
            drive_msg.drive.speed = error_based_velocities_["medium"];
        }
        else
        {
            drive_msg.drive.speed = error_based_velocities_["low"];
        }
        drive_pub_->publish(drive_msg);




        //float smooth_angle = SMOOTHINH_FACTOR * previous_angle + (1 - SMOOTHINH_FACTOR) * angle;
        //previous_angle = smooth_angle;
    

        // 8. Crear y publicar el mensaje de control
        //auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        //drive_msg.header.stamp = this->get_clock()->now();
        //drive_msg.drive.steering_angle = smooth_angle;
        //drive_msg.drive.speed = 1;  // podés ajustar la velocidad si querés

        //RCLCPP_INFO(this->get_logger(), "Gap start: %d, end: %d", gap_start, gap_end);
        //RCLCPP_INFO(this->get_logger(), "Best index: %d, angle: %.2f deg, distance: %.2f", best_index, angle * 180 / M_PI, ranges[best_index]);

        //drive_pub_->publish(drive_msg);
    }

    
    

};


int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}