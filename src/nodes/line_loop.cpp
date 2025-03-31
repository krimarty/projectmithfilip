//
// Created by student on 24.3.25.
//
#include "nodes/line_loop.h"
#include "algorithms/kinematics_algorithms.h"

namespace nodes {
    LineLoop::LineLoop() : Node("line_node"){
        // Initialize the subscriber
        line_sensors_subscriber_ = create_subscription<std_msgs::msg::UInt16MultiArray>(
            "/bpc_prp_robot/line_sensors", 1, std::bind(&LineLoop::on_line_sensors_msg, this, std::placeholders::_1));
        motorSpeed_publisher_ = create_publisher<std_msgs::msg::UInt8MultiArray>("/bpc_prp_robot/set_motor_speeds", 1);

        //DrillLogger_ = std::make_shared<DrillLogger>();
        //pid_ = std::make_shared<algorithms::Pid>(50*0.6,0.5*16,0.5);    }
        //pid_ = std::make_shared<algorithms::Pid>(50*0.9,0.4,0.7);    }
        //pid_ = std::make_shared<algorithms::Pid>(40,0.6,0);    } //osmicka
        pid_ = std::make_shared<algorithms::Pid>(35,0.6,0);    }



    float LineLoop::estimate_continuous_line_pose(const float left_value, const float right_value){
        return (right_value - left_value) / LSM_A;
    }

    DiscreteLinePose LineLoop::estimate_discrete_line_pose(float l_norm, float r_norm)
    {
        if (l_norm > 0.5 && r_norm > 0.5)
            return DiscreteLinePose::LineBoth;
        else if (l_norm > 0.5)
            return DiscreteLinePose::LineOnLeft;
        else if (r_norm > 0.5)
            return DiscreteLinePose::LineOnRight;
        else
            return DiscreteLinePose::LineNone;
    }

    void LineLoop::publish_motorSpeed(const double l, const double r) { //Je potreba udelat prepocet rychlosti na cislo do enkoderu
        auto msg = std_msgs::msg::UInt8MultiArray();
        msg.data.resize(2);

        msg.data[0] = 127 + l * 10;
        msg.data[1] = 127 + r * 10;

        motorSpeed_publisher_->publish(msg);
        //RCLCPP_INFO(get_logger(), "Published: %d", msg.data[0]);
    }

    void LineLoop::line_loop_timer_callback(){


        RobotSpeed robot_speed{};


        nodes::DiscreteLinePose tmp = estimate_discrete_line_pose(l_sensor, r_sensor);;


        /*
        if (tmp == nodes::DiscreteLinePose::LineNone){
            std::cout << "DiscreteLinePose::LineNone" << std::endl;
            robot_speed.w = 0;
        }
        //if (tmp == nodes::DiscreteLinePose::LineBoth)
        else if (tmp == nodes::DiscreteLinePose::LineOnLeft){
            std::cout << "DiscreteLinePose::LineOnLeft" << std::endl;
            robot_speed.w = 0.18;
        }
        else if (tmp == nodes::DiscreteLinePose::LineOnRight){
            std::cout << "DiscreteLinePose::LineOnRight" << std::endl;
            robot_speed.w = -0.18;
        }
        robot_speed.v = 0.04;

        WheelSpeed wheel_speed = algorithms::KinematicsAlgorithms::Inverse_kinematics(robot_speed);
        publish_motorSpeed(wheel_speed.l, wheel_speed.r);
        */


        robot_speed.w = pid_->step(estimate_continuous_line_pose(l_sensor, r_sensor)/1000, 0.01);
        //robot_speed.v = 0.05;
        robot_speed.v = 0.035;

        std::cout << l_sensor << " " << r_sensor << std::endl;
        WheelSpeed wheel_speed = algorithms::KinematicsAlgorithms::Inverse_kinematics(robot_speed);
        publish_motorSpeed(wheel_speed.l, wheel_speed.r);

    }
}
