//
// Created by student on 10.3.25.
//

#ifndef ENCODER_NODE_H
#define ENCODER_NODE_H

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <atomic>
#include <utility>


namespace nodes {
    class EncoderNode : public rclcpp::Node {
    public:
        // Constructor
        EncoderNode();
        // Destructor (default)
        ~EncoderNode() override = default;


        int get_left_value();
        int get_right_value();
        //Function to retireve the last pressed button value
        //int get_button_pressed() const;
        //void publish_message(int option);

    private:
        // Variable to store the last received button press value
        std::atomic<int> encoderValue0_ = 0;
        std::atomic<int> encoderValue1_ = 0;


        // Subscriber for button press messages
        rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr encoders_subscriber_;

        //rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr rgb_publisher_;




        // Callback - preprocess received message
        void encoder_callback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg)
        {
            encoderValue0_ = msg->data[0];
            encoderValue1_ = - msg->data[1];
            //std::cout << "encoderValue_: " << encoderValue0_ << ", " << encoderValue1_ << std::endl;
        }
    };
}

#endif //ENCODER_NODE_H
