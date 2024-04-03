#include "arduino_comms.h"
#include "wheel.h"
#include <algorithm>
#include <cstdint>
#include <iostream>
#include <chrono>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#define RAD2DEG(x) ((x)*180./M_PI)
using namespace std::chrono_literals;
using std::placeholders::_1;

uint8_t state = 0;

class WallbeBot : public rclcpp::Node
{
    public:
        geometry_msgs::msg::Twist twist;

        WallbeBot()
        : Node("wallbe_bot")
        {
            pub_  = this->create_publisher<geometry_msgs::msg::Twist>(
            "/diffbot_base_controller/cmd_vel_unstamped", 10);

            sub_  = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&WallbeBot::LiDARHandler, this, _1));

            timer_ = this->create_wall_timer(
            500ms, std::bind(&WallbeBot::checkState, this));

        }
    private:
        void move(float speed, float turn);
        void checkState();
        void LiDARHandler(sensor_msgs::msg::LaserScan::SharedPtr scan);

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
};

void WallbeBot::move(float speed, float turn) {
    this->twist.linear.x = speed;
    this->twist.linear.y = 0;
    this->twist.linear.z = 0;

    this->twist.angular.x = 0;
    this->twist.angular.y = 0;
    this->twist.angular.z = turn;
}

void WallbeBot::checkState() {
    switch(state) {
        case 0: // No obstacles detected
            std::cout << "Clear, keep moving forward" << std::endl;
            this->move(0.15, 0);
            break;
        case 1: // Only left is blocked
            std::cout << "Left is blocked" << std::endl;
            break;
        case 2: // Only right is blocked
            std::cout << "Right is blocked" << std::endl;
            break;
        case 3: // Both left and right is blocked
            std::cout << "Both left and right are blocked" << std::endl;
            break;
        case 4: // Forward is blocked
            std::cout << "Forward is blocked" << std::endl;
            break;
        case 5: // Forward and Left is blocked
            break;
        case 6: // Forward and Right is blocked
            break;
        case 7: // Left, Right and Forward is blocked
            break;
        default:
            std::cout << "Unknown state!!!" << std::endl;
            break;
    }
    pub_->publish(this->twist);
}

void WallbeBot::LiDARHandler(sensor_msgs::msg::LaserScan::SharedPtr scan) {
    /*
     * The car has 4 state:
     *  - 0: No obstacles
     *  - 1: Left is blocked 
     *  - 2: Right is blocked 
     *  - 3: Bot Left and Right is blocked 
     *  - 4: Forward is blocked
     *  - 5: Forward and Left is blocked 
     *  - 6: Forward and Right is blocked 
     *  - 7: Forward, right and left blocked 
     */
    int count = scan->scan_time / scan->time_increment;
    state = 0;

    std::vector<std::pair<float, float>> lidar_scan(count, std::make_pair(0, 0));

    for (int i = 0; i < count; i++) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        float distance = scan->ranges[i];
        // lidar_scan[i].first = degree;
        // lidar_scan[i].second = distance;

        if (degree <= -65 && degree >= -115 && distance <= 0.25) {
            state |= 0x02; 
        }

        if (degree >= 65 && degree <= 115 && distance <= 0.25) {
            state |= 0x01; 
        }
        if (degree <= 45 && degree >= -45 && distance <= 0.3) {
            state |= 0x04; 
        }
    }
    //std::cout << "State value: " << +state << std::endl;
}



int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallbeBot>());
    rclcpp::shutdown();
    return 0;
}
