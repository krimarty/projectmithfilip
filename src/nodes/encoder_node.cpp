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

    std::pair<int, int> EncoderNode::get_values()
    {
        return std::make_pair(encoderValue0_.load(), encoderValue1_.load());
    }

    /*
     int MotorNode::get_button_pressed() const {
          //std::cout << "button_pressed_: " << button_pressed_ << std::endl;
          return button_pressed_;
      }
     */

    /*
    void EncoderNode::publish_message() {
        auto msg = std_msgs::msg::UInt32MultiArray();
        msg.data.resize(2);
        msg.data[0] = 69;
        msg.data[1] = 69;
        encoders_publisher_->publish(msg);
        //RCLCPP_INFO(get_logger(), "Published: %d", msg.data[0]);
    }
     */






}