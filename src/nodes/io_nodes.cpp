//
// Created by student on 3.3.25.
//

#include "nodes/io_node.h"

namespace nodes{
    IoNode::IoNode() : Node("io_node"){
        // Initialize the subscriber
        button_subscriber_ = create_subscription<std_msgs::msg::UInt8>(
            "/bpc_prp_robot/buttons", 1, std::bind(&IoNode::on_button_callback, this, std::placeholders::_1));

        rgb_publisher_ = create_publisher<std_msgs::msg::UInt8MultiArray>("/bpc_prp_robot/rgb_leds", 1);
    }

    int IoNode::get_button_pressed() const {
      //std::cout << "button_pressed_: " << button_pressed_ << std::endl;
        return button_pressed_;
    }

    void IoNode::publish_message(int option) {
        auto msg = std_msgs::msg::UInt8MultiArray();
        msg.data.resize(12);
        msg.data[0] = 30* option;
        msg.data[1] = 30* option;
        msg.data[2] = 30* option;
        msg.data[3] = 30* option;
        msg.data[4] = 30* option;
        msg.data[5] = 30* option;
        msg.data[6] = 30* option;
        msg.data[7] = 30* option;
        msg.data[8] = 30* option;
        msg.data[9] = 30* option;
        msg.data[10] = 30* option;
        msg.data[11] = 30* option;
        rgb_publisher_->publish(msg);
        //RCLCPP_INFO(get_logger(), "Published: %d", msg.data[0]);
        }




}
