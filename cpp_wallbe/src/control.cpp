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
uint8_t state = 0;

class ControlPublisher : public rclcpp::Node
{
    public:
        ControlPublisher()
        : Node("control_publisher")
        {
            publisher_  = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
            timer_ = this->create_wall_timer(
            500ms, std::bind(&ControlPublisher::timer_callback, this));
        }
    private:
        void turnLeft();
        void turnRight();
        void moveBackward();
        void checkState();

        void timer_callback() {
            checkState();
        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

void ControlPublisher::turnLeft() {
    std::cout << "We're turning to the left..." << std::endl;
}

void ControlPublisher::turnRight() {
    std::cout << "We're turning to the right..." << std::endl;
}

void ControlPublisher::moveBackward() {
    std::cout << "We're moving backward..." << std::endl;
}

void ControlPublisher::checkState() {
    switch(state) {
        case 0: // No obstacles detected
            std::cout << "Clear, keep moving forward" << std::endl;
            // setSpeed(20, 20);
            break;
        case 1: // Only left is blocked
            std::cout << "Left is blocked" << std::endl;
            // setSpeed(20, 20);
            break;
        case 2: // Only right is blocked
            std::cout << "Right is blocked" << std::endl;
            // setSpeed(20, 20);
            break;
        case 3: // Both left and right is blocked
            std::cout << "Both left and right are blocked" << std::endl;
            // setSpeed(20, 20);
            break;
        case 4: // Forward is blocked
            std::cout << "Forward is blocked" << std::endl;
            break;
        case 5: // Forward and Left is blocked
            this->turnRight();
            break;
        case 6: // Forward and Right is blocked
            this->turnLeft();
            break;
        case 7: // Left, Right and Forward is blocked
                this->moveBackward();
                break;
        default:
            std::cout << "Unknown state!!!" << std::endl;
            break;
    }
}

static void LiDARHandler(sensor_msgs::msg::LaserScan::SharedPtr scan) {
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
        std::cout << "Distance: " << distance << std::endl;
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

    auto node = rclcpp::Node::make_shared("rplidar_client");

    auto lidar_info_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
                    "scan", rclcpp::SensorDataQoS(), LiDARHandler);

    rclcpp::spin(node);
    rclcpp::spin(std::make_shared<ControlPublisher>());
    rclcpp::shutdown();

    return 0;
}
