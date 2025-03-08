#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <fstream>
#include <sstream>
#include <vector>

class TrajectoryReplayPublisher : public rclcpp::Node {
public:
    TrajectoryReplayPublisher() : Node("trajectory_replay_publisher") {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1),
            std::bind(&TrajectoryReplayPublisher::follow_trajectory, this));
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::pair<double, double>> trajectory_;
    size_t current_index_ = 0;

    void follow_trajectory() {
        if (trajectory_.empty()) {
            if (!load_trajectory()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to load trajectory!");
                return;
            }
        }

        if (current_index_ >= trajectory_.size()) {
            RCLCPP_INFO(this->get_logger(), "Trajectory finished!");
            return;
        }

        auto cmd_msg = geometry_msgs::msg::Twist();
        cmd_msg.linear.x = 0.8 * cmd_msg.linear.x + 0.2 * trajectory_[current_index_].first;
        cmd_msg.angular.z = 0.8 * cmd_msg.angular.z + 0.2 * trajectory_[current_index_].second;
        cmd_vel_pub_->publish(cmd_msg);
        RCLCPP_INFO(this->get_logger(), "Published cmd_vel: [%f, %f]", cmd_msg.linear.x, cmd_msg.angular.z);
        current_index_++;
    }

    bool load_trajectory() {
        std::ifstream file("trajectory.csv");
        if (!file.is_open()) {
            return false;
        }

        std::string line;
        std::getline(file, line); // Skip header

        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string value;
            std::pair<double, double> data;

            std::getline(ss, value, ',');
            std::getline(ss, value, ',');
            data.first = std::stod(value);
            std::getline(ss, value, ',');
            data.second = std::stod(value);
            trajectory_.push_back(data);
        }

        file.close();
        return true;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryReplayPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
