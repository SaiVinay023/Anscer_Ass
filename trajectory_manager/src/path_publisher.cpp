#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class PathPublisher : public rclcpp::Node {
public:
    PathPublisher() : Node("path_publisher") {
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/robot_path", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&PathPublisher::publish_path, this));
        RCLCPP_INFO(this->get_logger(), "Path Publisher Node Started");
    }

private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void publish_path() {
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = this->now();

        for (int i = 0; i < 5; i++) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = i * 0.5;
            pose.pose.position.y = i * 0.5;
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }

        path_pub_->publish(path_msg);
        RCLCPP_INFO(this->get_logger(), "Published path with %ld poses", path_msg.poses.size());
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
