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
        int blue, red;
        if (option > 0)
        {
            blue = 0;
            red = 1;
        }
        else
        {
            blue = 1;
            red = 0;
        }

        msg.data.resize(12);
        //red
        msg.data[0] = 0;
        msg.data[1] = 0;
        msg.data[2] = 255 * blue;
        //red
        msg.data[6] = 255* red;
        msg.data[7] = 0;
        msg.data[8] = 0;
        //blue
        msg.data[3] = 55* red;
        msg.data[4] = 0;
        msg.data[5] = 0;
        //blue
        msg.data[9] = 0;
        msg.data[10] = 0;
        msg.data[11] = 255 * blue;
        rgb_publisher_->publish(msg);
        //RCLCPP_INFO(get_logger(), "Published: %d", msg.data[0]);
        }




}
