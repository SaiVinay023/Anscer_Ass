#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <fstream>
#include <sstream>

class TrajectoryReaderPublisher : public rclcpp::Node {
public:
    TrajectoryReaderPublisher() : Node("trajectory_reader_publisher") {
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/robot_path", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(5), std::bind(&TrajectoryReaderPublisher::publish_trajectory, this));
    }

private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void publish_trajectory() {
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "map";  

        std::ifstream file("trajectory.csv");
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open trajectory file!");
            return;
        }

        std::string line;
        std::getline(file, line);  // Skip header

        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string value;
            geometry_msgs::msg::PoseStamped pose;
            std::getline(ss, value, ',');
            pose.header.stamp.sec = std::stoi(value);  
            std::getline(ss, value, ',');
            pose.pose.position.x = std::stod(value);
            std::getline(ss, value, ',');
            pose.pose.position.y = std::stod(value);
            std::getline(ss, value, ',');
            pose.pose.position.z = std::stod(value);

            path_msg.poses.push_back(pose);
        }

        file.close();
        path_pub_->publish(path_msg);
        RCLCPP_INFO(this->get_logger(), "Published trajectory from file");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryReaderPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
