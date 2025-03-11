//
// Created by student on 10.3.25.
//
#include "nodes/encoder_node.h"

namespace nodes{
    EncoderNode::EncoderNode() : Node("encoder_node"){
        // Initialize the subscriber
        encoders_subscriber_ = create_subscription<std_msgs::msg::UInt32MultiArray>(
        "/bpc_prp_robot/encoders", 1, std::bind(&EncoderNode::encoder_callback, this, std::placeholders::_1));

        //encoders_publisher_ = create_publisher<std_msgs::msg::UInt8MultiArray>("/bpc_prp_robot/set_motor_encoders", 1);
    }

    int EncoderNode::get_left_value()
    {
        return encoderValue0_.load();
    }

    int EncoderNode::get_right_value()
    {
        return encoderValue1_.load();
    }

}