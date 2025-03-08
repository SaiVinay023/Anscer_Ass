#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <fstream>
#include <sstream>
#include <rclcpp/qos.hpp>  


class TrajectoryReaderPublisher : public rclcpp::Node {
public:
    TrajectoryReaderPublisher() : Node("trajectory_reader_publisher"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
        // Publisher for visualization in RViz
        rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local();
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/trajectory_markers", qos_profile);        
        // Read and publish trajectory on startup
        read_and_publish_trajectory("trajectory.csv");

        RCLCPP_INFO(this->get_logger(), "Trajectory Reader and Publisher Node Started");
    }

private:
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    void read_and_publish_trajectory(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", filename.c_str());
            return;
        }

        visualization_msgs::msg::MarkerArray marker_array;
        std::string line;
        int id = 0;

        while (std::getline(file, line)) {
            std::istringstream ss(line);
            double x, y, z;
            char comma;
            if (!(ss >> x >> comma >> y >> comma >> z)) {
                RCLCPP_ERROR(this->get_logger(), "Invalid line format in file: %s", line.c_str());
                continue;
            }

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "odom";  // Assume odom frame for visualization
            marker.header.stamp = this->now();
            marker.ns = "trajectory";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = x;
            marker.pose.position.y = y;
            marker.pose.position.z = z;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            marker_array.markers.push_back(marker);
        }

        marker_pub_->publish(marker_array);
        file.close();
        RCLCPP_INFO(this->get_logger(), "Published trajectory markers from file: %s", filename.c_str());
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryReaderPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
