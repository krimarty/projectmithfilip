//
// Created by student on 7.4.25.
//

#include <nodes/lidar_node.h>

namespace nodes{
    LidarNode::LidarNode() : Node("lidar_node"){

        lidar_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
           "/bpc_prp_robot/lidar", 1, std::bind(&LidarNode::lidar_callback, this, std::placeholders::_1));
    }
}
