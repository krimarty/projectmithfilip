//
// Created by martin on 16.04.25.
//
//
// Created by student on 3.3.25.
//

#include "nodes/coridor_node.h"

namespace nodes{
    CorridorNode::CorridorNode() : Node("corridor_node"),
       pid_coridor_center(1.0, 0.0, 0.3),
       pid_coridor_angle(1.0, 0.002, 0.3),
       pid_imu(0.5, 0.004, 0.0),
        wheel_speed(),
        robot_speed()


    {
        imu_class = std::make_shared<nodes::ImuNode>();
        motor_class = std::make_shared<nodes::MotorNode>();
        lidar_class = std::make_shared<nodes::LidarNode>();
    }

    void CorridorNode::corridor_routine()
    {
        current_state = next_state(current_state);

        switch (current_state)
        {
            case states::calibration:
                state_calibration();
                break;

            case states::corridor_following:
                state_corridor();
                break;

            default:
                break;
        }

        wheel_speed = algorithms::KinematicsAlgorithms::Inverse_kinematics(robot_speed);
        motor_class->publish_motorSpeed(wheel_speed.l, wheel_speed.r);
    }

    states CorridorNode::next_state(const states currentState)
    {
        if (currentState == states::calibration)
        {
            if (imu_class->calibrated_)
                return states::corridor_following;
            return states::calibration;
        }

        return states::calibration;
    }

    void CorridorNode::state_calibration() {}

    void CorridorNode::state_corridor()
    {
        if (lidar_class->from_centre() < 0.01)
        {
            robot_speed.w = pid_coridor_angle.step(lidar_class->lines.leftFront.iK, 0.01);;
            std::cout << "Uhluju" << std::endl;
        }
        else
        {
            robot_speed.w = pid_coridor_center.step(lidar_class->get_error(nodes::leftFront), 0.01);
            std::cout << "Centruju" << std::endl;
        }

        robot_speed.v = 0.055;
    }



}
