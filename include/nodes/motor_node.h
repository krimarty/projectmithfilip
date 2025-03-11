//
// Created by student on 10.3.25.
//

#ifndef MOTOR_NODE_H
#define MOTOR_NODE_H

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

namespace nodes {
    class MotorNode : public rclcpp::Node {
    public:
        // Constructor
        MotorNode();
        // Destructor (default)
        ~MotorNode() override = default;


        // Function to retireve the last pressed button value
        void publish_motorSpeed(double l, double r);

    private:
        // Variable to store the last received button press value
        int button_pressed_ = -1;

        // Subscriber for button press messages
        //rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr button_subscriber_;

        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motorSpeed_publisher_;




        // Callback - preprocess received message
        void on_button_callback(const std_msgs::msg::UInt8::SharedPtr msg)
        {
            button_pressed_ = msg->data;
            std::cout << "button_pressed_: " << button_pressed_ << std::endl;
        }
    };
}

#endif //MOTOR_NODE_H
