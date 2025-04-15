//
// Created by student on 7.4.25.
//

#ifndef LIDAR_NODE_H
#define LIDAR_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <algorithms/lidar_algorithms.h>

namespace nodes
{
    class LidarNode : public rclcpp::Node {
    public:

        LidarNode();

        ~LidarNode() override = default;

        float get_result() const
        {
            return results.left-results.right;
        }


    private:
        algorithms::LidarFiltr filtr;

        algorithms::LidarFiltrResults results{};
        algorithms::LidarLines lines{};

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;

        void lidar_callback(std::shared_ptr<sensor_msgs::msg::LaserScan> msg)
        {
            results = algorithms::LidarFiltr::apply_filter(msg->ranges, msg->angle_min, msg->angle_max);
            if (std::isnan(results.left))
                results.left = 0;
            if (std::isnan(results.right))
                results.right = 0;

            lines = filtr.line_aprox(msg->ranges, msg->angle_min, msg->angle_max);
            std::cout << "Leva predni " << "y=" << lines.leftFront.iK << "x + " << lines.leftFront.iQ << std::endl;
            std::cout << "Prava predni " << "y=" << lines.rightFront.iK << "x + " << lines.rightFront.iQ << std::endl;
            std::cout << "Leva zadni " << "y=" << lines.leftBack.iK << "x + " << lines.leftBack.iQ << std::endl;
            std::cout << "Prava zadni " << "y=" << lines.rightBack.iK << "x + " << lines.rightBack.iQ << std::endl;
            }


    };
}

#endif //LIDAR_NODE_H
