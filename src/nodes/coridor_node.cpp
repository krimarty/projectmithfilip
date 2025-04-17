//
// Created by martin on 16.04.25.
//
//
// Created by student on 3.3.25.
//

#include "nodes/coridor_node.h"

namespace nodes{
    CorridorNode::CorridorNode() : Node("corridor_node"),
       pid_coridor_center(2.0, 0.0, 0.4),
       pid_coridor_angle(1.0, 0.002, 0.2),
       pid_moveToTargetAhead(0.5, 0.002, 0.0),
       pid_imu(1, 0.004, 0.0),
        wheel_speed(),
        robot_speed()


    {
        imu_class = std::make_shared<nodes::ImuNode>();
        motor_class = std::make_shared<nodes::MotorNode>();
        lidar_class = std::make_shared<nodes::LidarNode>();
        io_class = std::make_shared<nodes::IoNode>();
    }

    void CorridorNode::corridor_routine()
    {
        //////////////////////////////
        intersectionType tmp = lidar_class->get_interseptionType();
        if (tmp == middleX)
            io_class->publish_message(4);
        else if (tmp == leftTurn)
            io_class->publish_message(2);
        else if (tmp == rightTurn)
            io_class->publish_message(1);
        else if (tmp == blindEnd)
            io_class->publish_message(0);
        else if (tmp == straightCorridor)
            io_class->publish_message(3);
        //////////////////////////////
        current_state = next_state(current_state);

        switch (current_state)
        {
            case states::calibration:
                state_calibration();
                break;

            case states::corridor_following:
                state_corridor();
                break;

            case states::turningLeft:
                state_left();
                break;

            case states::turningRight:
                state_right();
                break;

            case states::center:
                state_center();
                break;

            default:
                break;
        }

        wheel_speed = algorithms::KinematicsAlgorithms::Inverse_kinematics(robot_speed);
        //motor_class->publish_motorSpeed(wheel_speed.l, wheel_speed.r);
    }

    states CorridorNode::next_state(const states currentState)
    {
        if (currentState == states::calibration)
        {
            if (imu_class->calibrated_)
                return states::center;
            return states::calibration;
        }

        return states::calibration;
    }

    void CorridorNode::state_calibration() {}

    void CorridorNode::state_corridor()
    {
        if (lidar_class->from_centre() < 0.01)
        {
            robot_speed.w = pid_coridor_angle.step(lidar_class->get_error_angle(line), 0.01);
            pid_coridor_center.step(lidar_class->get_error_distance(line), 0.01);
            //std::cout << "Uhluju" << std::endl;
        }
        else
        {
            pid_coridor_angle.step(lidar_class->get_error_angle(line), 0.01);
            robot_speed.w = pid_coridor_center.step(lidar_class->get_error_distance(line), 0.01);
            //std::cout << "Centruju" << std::endl;
        }

        robot_speed.v = 0.1;
    }

    void CorridorNode::state_left()
    {
        switch (current_turning_state)
        {
        case turning_states::moveToTargetAhead:
            robot_speed.w = pid_imu.step(imu_class->planar_integrator_.getYaw(), 0.01);
            robot_speed.v = pid_moveToTargetAhead.step( lidar_class->from_straight()-0.18, 0.01);
            std::cout << lidar_class->from_straight() << std::endl;
            if (lidar_class->from_straight() < 0.21)
                current_turning_state = turning_states::turning;
            break;

        case turning_states::turning:
            robot_speed.v = 0;
            //robot_speed.w = 0;
            robot_speed.w = pid_imu.step( imu_class->planar_integrator_.getYaw() + M_PI/2, 0.01);
            if (imu_class->planar_integrator_.getYaw() > M_PI/2 - 0.0174532925 && imu_class->planar_integrator_.getYaw() < M_PI/2 + 0.0174532925 ) // bulharka na urcenni pm 5 stupnu
                current_turning_state = turning_states::moveToTargetAhead;
            break;
        }
    }

    void CorridorNode::state_right()
    {
        switch (current_turning_state)
        {
        case turning_states::moveToTargetAhead:
            robot_speed.w = pid_imu.step(imu_class->planar_integrator_.getYaw(), 0.01);
            robot_speed.v = pid_moveToTargetAhead.step( lidar_class->from_straight()-0.18, 0.01);
            std::cout << lidar_class->from_straight() << std::endl;
            if (lidar_class->from_straight() < 0.21)
                current_turning_state = turning_states::turning;
            break;

        case turning_states::turning:
            robot_speed.v = 0;
            //robot_speed.w = 0;
            robot_speed.w = pid_imu.step( imu_class->planar_integrator_.getYaw() - M_PI/2, 0.01);
            if (imu_class->planar_integrator_.getYaw() < -M_PI/2 + 0.0174532925 && imu_class->planar_integrator_.getYaw() > - M_PI/2 - 0.0174532925 ) // bulharka na urcenni pm 5 stupnu
                current_turning_state = turning_states::moveToTargetAhead;
            break;
        }


    }

    void CorridorNode::state_center()
    {
        robot_speed.w = pid_imu.step(imu_class->planar_integrator_.getYaw(), 0.01);
        robot_speed.v = 0.1;
    }



}
