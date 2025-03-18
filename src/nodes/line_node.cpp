//
// Created by student on 17.3.25.
//
#include "nodes/line_node.h"

namespace nodes {
    LineNode::LineNode() : Node("line_node"){
        // Initialize the subscriber
        line_sensors_subscriber_ = create_subscription<std_msgs::msg::UInt16MultiArray>(
            "/bpc_prp_robot/line_sensors", 1, std::bind(&LineNode::on_line_sensors_msg, this, std::placeholders::_1));

    }

    float LineNode::estimate_continuous_line_pose(const float left_value, const float right_value){
        return (right_value - left_value) / LSM_A;
    }

    DiscreteLinePose LineNode::estimate_discrete_line_pose(float l_norm, float r_norm)
    {
        if (l_norm > 0.5 && r_norm > 0.5)
            return DiscreteLinePose::LineBoth;
        else if (l_norm > 0.5)
            return DiscreteLinePose::LineOnLeft;
        else if (r_norm > 0.5)
            return DiscreteLinePose::LineOnRight;
        else
            return DiscreteLinePose::LineNone;
    }
}
