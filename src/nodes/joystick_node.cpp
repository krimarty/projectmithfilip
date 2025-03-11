#include "nodes/joystick_node.h"
//
// Created by martin on 11.03.25.
//
namespace nodes{
    JoystickNode::JoystickNode() : Node("joystick_node"){
        // Initialize the subscriber
        joy_subscriber_ = create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 1, std::bind(&JoystickNode::joystick_callback, this, std::placeholders::_1));
    }

    float JoystickNode::get_w_() const
    {
        return dir_turn_;
    }
    float JoystickNode::get_v_() const
    {
        if (dir_forward_ < 1)
            return -(dir_forward_ - 1)/6;
        else
            return (dir_backward_ - 1)/6;
    }

}