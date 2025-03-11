//
// Created by student on 10.3.25.
//
#include "nodes/motor_node.h"

namespace nodes{
    MotorNode::MotorNode() : Node("motor_node"){
        // Initialize the subscriber
        //button_subscriber_ = create_subscription<std_msgs::msg::UInt8>(
        //    "/bpc_prp_robot/buttons", 1, std::bind(&MotorNode::on_button_callback, this, std::placeholders::_1));

        motorSpeed_publisher_ = create_publisher<std_msgs::msg::UInt8MultiArray>("/bpc_prp_robot/set_motor_speeds", 1);
    }

   /*
    int MotorNode::get_button_pressed() const {
         //std::cout << "button_pressed_: " << button_pressed_ << std::endl;
         return button_pressed_;
     }
    */

    void MotorNode::publish_motorSpeed(const double l, const double r) { //Je potreba udelat prepocet rychlosti na cislo do enkoderu
        auto msg = std_msgs::msg::UInt8MultiArray();
        msg.data.resize(2);

        msg.data[0] = 127 + l * 10;
        msg.data[1] = 127 + r * 10;

        motorSpeed_publisher_->publish(msg);
        //RCLCPP_INFO(get_logger(), "Published: %d", msg.data[0]);
    }




}