//
// Created by martin on 11.03.25.
//

#ifndef JOYSTICK_NODE_H
#define JOYSTICK_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace nodes {
    class JoystickNode : public rclcpp::Node {
    public:
        // Constructor
        JoystickNode();
        // Destructor (default)
        ~JoystickNode() override = default;


        // Function to retireve the last pressed button value
        float get_w_() const;
        float get_v_() const;


    private:
        // Variable to store the last received button press value
        std::atomic<float> dir_turn_ = 0;
        std::atomic<float> dir_forward_ = 0;
        std::atomic<float> dir_backward_ = 0;

        // Subscriber for button press messages
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;

        // Callback - preprocess received message
        void joystick_callback(const sensor_msgs::msg::Joy msg)
        {
            dir_turn_ = msg.axes[0];
            dir_forward_ = msg.axes[5];
            dir_backward_ = msg.axes[2];
        }
    };
}

#endif //JOYSTICK_NODE_H
