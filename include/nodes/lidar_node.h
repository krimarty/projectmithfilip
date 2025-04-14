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
        algorithms::LidarFiltrResults results{};

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;

        void lidar_callback(std::shared_ptr<sensor_msgs::msg::LaserScan> msg)
        {
            results = algorithms::LidarFiltr::apply_filter(msg->ranges, msg->angle_min, msg->angle_max);
            if (std::isnan(results.left))
                results.left = 0;
            if (std::isnan(results.right))
                results.right = 0;
            }
    };
}

#endif //LIDAR_NODE_H
