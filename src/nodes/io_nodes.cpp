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
        msg.data[0] = 0;
        msg.data[1] = 0;
        msg.data[2] = 0;
        msg.data[6] = 0;
        msg.data[7] = 0;
        msg.data[8] = 0;
        msg.data[3] = 0;
        msg.data[4] = 0;
        msg.data[5] = 0;
        msg.data[9] = 0;
        msg.data[10] = 0;
        msg.data[11] = 0;



        if (option == 0)
        {
            msg.data[1] = 80;//zadni

        }
        else if (option == 1)
        {
            msg.data[4] = 80;//prava

        }
        else if (option == 2)
        {
            msg.data[7] = 80;//leva

        }
        else if (option == 3)
        {
            msg.data[10] = 80;//predni
        }
        else if (option == 4)
        {
            msg.data[1] = 80;//vsecky
            msg.data[4] = 80;
            msg.data[7] = 80;
            msg.data[10] = 80;
        }
        else if (option == 5)
        {
            msg.data[7] = 80; // T strany
            msg.data[4] = 80;
        }
        else if (option == 6)
        {
            msg.data[7] = 80; // T predni, leva
            msg.data[10] = 80;
        }
        else if (option == 7)
        {
            msg.data[10] = 80; // T predni, prva
            msg.data[4] = 80;
        }

        rgb_publisher_->publish(msg);
        //RCLCPP_INFO(get_logger(), "Published: %d", msg.data[0]);
        }




}
