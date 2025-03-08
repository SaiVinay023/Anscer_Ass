#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <fstream>
#include "trajectory_manager/srv/save_trajectory.hpp"
#include <rclcpp/qos.hpp> 

using std::placeholders::_1;
using std::placeholders::_2;

class TrajectoryPublisherSaver : public rclcpp::Node {
public:
    TrajectoryPublisherSaver() 
        : Node("trajectory_publisher_saver"), 
          tf_buffer_(this->get_clock()), 
          tf_listener_(tf_buffer_), 
          duration_(30.0)  // Default duration of 30 seconds
    {
        // Subscribe to Odometry
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::SensorDataQoS(),  
            std::bind(&TrajectoryPublisherSaver::odom_callback, this, std::placeholders::_1)
        );

        // Publisher for visualization
        rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local(); 
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/trajectory_markers", qos_profile);

        // ROS 2 Service to save trajectory
        save_service_ = this->create_service<trajectory_manager::srv::SaveTrajectory>(
            "save_trajectory", 
            std::bind(&TrajectoryPublisherSaver::save_trajectory_callback, this, _1, _2)
        );

        RCLCPP_INFO(this->get_logger(), "Trajectory Publisher and Saver Node Started");
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Service<trajectory_manager::srv::SaveTrajectory>::SharedPtr save_service_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::vector<geometry_msgs::msg::PoseStamped> trajectory_;
    double duration_;  // Store the duration for trajectory trimming

    // Callback for Odometry messages
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = msg->header;
        pose.pose = msg->pose.pose;
        
        trajectory_.push_back(pose);

        // Remove old points that exceed the duration
        double current_time = this->now().seconds();
        while (!trajectory_.empty() && 
               (current_time - trajectory_.front().header.stamp.sec > duration_)) 
        {
            trajectory_.erase(trajectory_.begin());
        }

        publish_markers();
    }

    // Function to publish markers for visualization
    void publish_markers() {
        visualization_msgs::msg::MarkerArray marker_array;
        int id = 0;

        for (const auto &pose : trajectory_) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";  // Adjust if necessary
            marker.header.stamp = this->now();
            marker.ns = "trajectory";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose = pose.pose;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            marker_array.markers.push_back(marker);
        }

        marker_pub_->publish(marker_array);
    }

    // Service callback to save trajectory to a CSV file
    bool save_trajectory_callback(
        const std::shared_ptr<trajectory_manager::srv::SaveTrajectory::Request> request,
        std::shared_ptr<trajectory_manager::srv::SaveTrajectory::Response> response) 
    {
        std::ofstream file(request->filename);
        if (!file.is_open()) {
            response->success = false;
            response->message = "Failed to open file!";
            RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", request->filename.c_str());
            return false;
        }

        file << "timestamp,x,y,z\n";  // Header for CSV
        rclcpp::Time now = this->now();
        
        for (const auto &pose : trajectory_) {
            if ((now - pose.header.stamp).seconds() <= request->duration) {
                file << pose.header.stamp.sec << ","
                     << pose.pose.position.x << ","
                     << pose.pose.position.y << ","
                     << pose.pose.position.z << "\n";
            }
        }

        file.close();
        response->success = true;
        response->message = "Trajectory saved successfully!";
        RCLCPP_INFO(this->get_logger(), "Trajectory saved to %s", request->filename.c_str());
        return true;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryPublisherSaver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
