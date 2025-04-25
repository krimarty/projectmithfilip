//
// Created by martin on 16.04.25.
//
//
// Created by student on 3.3.25.
//

#include "nodes/maze_node.h"

namespace nodes{
    MazeNode::MazeNode() : Node("maze_node"),
       pid_coridor_center(2.0, 0.0, 0.4),
       pid_coridor_angle(1.0, 0.002, 0.2),
       pid_moveToTargetAhead(0.5, 0.002, 0.0),
       pid_imu(1, 0.004, 0.0),
        wheel_speed(),
        robot_speed()


    {
        camera_class = std::make_shared<nodes::CameraNode>();
        imu_class = std::make_shared<nodes::ImuNode>();
        motor_class = std::make_shared<nodes::MotorNode>();
        lidar_class = std::make_shared<nodes::LidarNode>();
        io_class = std::make_shared<nodes::IoNode>();
        encoder_class = std::make_shared<nodes::EncoderNode>();
    }

    void MazeNode::maze_routine()
    {
        //////////////////////////////
        intersectionType tmp = lidar_class->get_intersection();
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
        else if (tmp == T)
            io_class->publish_message(5);
        else if (tmp == TLeft)
            io_class->publish_message(6);
        else if (tmp == TRight)
            io_class->publish_message(7);
        //////////////////////////////

        line_select();
        update_coordinates();
        std::cout << "x: " << coordinates.x << " y: " << coordinates.y << std::endl;

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
                //state_center();
                intersection_handle();
                break;

            default:
                break;
        }

        wheel_speed = algorithms::KinematicsAlgorithms::Inverse_kinematics(robot_speed);
        motor_class->publish_motorSpeed(wheel_speed.l, wheel_speed.r);
    }

    states MazeNode::next_state(const states currentState)
    {
        intersectionType tmp = lidar_class->get_intersection();
        if (currentState == states::calibration)
        {
            if (imu_class->calibrated_)
                return states::corridor_following;
            return states::calibration;
        }
        else if (currentState == states::corridor_following) {
            if (tmp == middleX)
                return states::center;
            else if (tmp == leftTurn)
                return states::turningLeft;
            else if (tmp == rightTurn)
                return states::turningRight;
            return states::corridor_following;
        }
        else if (currentState == states::turningLeft) {
            if (current_turning_state == finished) {
                current_turning_state = moveToTargetAhead;
                return states::corridor_following;
            }
            return states::turningLeft;
        }
        else if (currentState == states::turningRight) {
            if (current_turning_state == finished) {
                current_turning_state = moveToTargetAhead;
                return states::corridor_following;
            }
            return states::turningRight;
        }
        else if (currentState == states::center) {
            if (current_intersection_state == intersectionFinished)
                return states::corridor_following;
            return states::center;
        }
        return states::calibration;
    }

    void MazeNode::state_calibration()
    {
        reset_coordinates();
    }

    void MazeNode::state_corridor()
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

        if (0.02 > abs(lidar_class->get_error_angle(line))) {
            //std::cout << "K: " << std::abs(lidar_class->get_error_angle(line)) << "yaw: " << imu_class->planar_integrator_.getYaw() << std::endl;
            imu_class->planar_integrator_.reset();
            //std::cout << "yaw after reset:" << imu_class->planar_integrator_.getYaw() << std::endl;

        }
    }

    void MazeNode::state_left()
    {
        switch (current_turning_state)
        {
        case turning_states::moveToTargetAhead:
            robot_speed.w = pid_imu.step(imu_class->planar_integrator_.getYaw(), 0.01);
            robot_speed.v = pid_moveToTargetAhead.step( lidar_class->from_straight()-0.18, 0.01);
            //std::cout << lidar_class->from_straight() << std::endl;
            if (lidar_class->from_straight() < 0.21)
                current_turning_state = turning_states::turning;
            break;

        case turning_states::turning:
            robot_speed.v = 0;
            //robot_speed.w = 0;
            robot_speed.w = pid_imu.step( imu_class->planar_integrator_.getYaw() + M_PI/2, 0.01);
            if (imu_class->planar_integrator_.getYaw() > -M_PI/2 - 0.0698131701 && imu_class->planar_integrator_.getYaw() < -M_PI/2 + 0.0698131701 ) // bulharka na urcenni pm 5 stupnu
                current_turning_state = turning_states::resetImu;
            break;

        case turning_states::resetImu:
            robot_speed.w = pid_coridor_angle.step(lidar_class->get_error_angle(right), 0.01);
            robot_speed.v = 0;
            if (lidar_class->get_error_angle(right) < 0.01)
            {
                imu_class->planar_integrator_.reset();
                current_turning_state = turning_states::finished;
            }

        default:
            robot_speed.w = 0;
            imu_class->planar_integrator_.reset();
            break;
        }
    }

    void MazeNode::state_right()
    {
        switch (current_turning_state)
        {
        case turning_states::moveToTargetAhead:
            robot_speed.w = pid_imu.step(imu_class->planar_integrator_.getYaw(), 0.01);
            robot_speed.v = pid_moveToTargetAhead.step( lidar_class->from_straight()-0.18, 0.01);
            //std::cout << lidar_class->from_straight() << std::endl;
            if (lidar_class->from_straight() < 0.21)
                current_turning_state = turning_states::turning;
            break;

        case turning_states::turning:
            robot_speed.v = 0;
            //robot_speed.w = 0;
            robot_speed.w = pid_imu.step( imu_class->planar_integrator_.getYaw() - M_PI/2, 0.01);
            if (imu_class->planar_integrator_.getYaw() < M_PI/2 + 0.0698131701 && imu_class->planar_integrator_.getYaw() > M_PI/2 - 0.0698131701 ) // bulharka na urcenni pm 5 stupnu
                current_turning_state = turning_states::resetImu;
            break;

        case turning_states::resetImu:
            robot_speed.w = pid_coridor_angle.step(lidar_class->get_error_angle(left), 0.01);
            robot_speed.v = 0;
            if (lidar_class->get_error_angle(left) < 0.01)
            {
                imu_class->planar_integrator_.reset();
                current_turning_state = turning_states::finished;
            }

        default:
                robot_speed.w = 0;
                imu_class->planar_integrator_.reset();
            break;
        }


    }

    void MazeNode::state_center()
    {
        robot_speed.w = pid_imu.step(imu_class->planar_integrator_.getYaw(), 0.01);
        robot_speed.v = 0.1;
    }

    void MazeNode::intersection_handle()
    {
        switch (current_intersection_state)
        {
        case intersectionStates::resetCoordinates:
            reset_coordinates();
            current_intersection_state = intersectionStates::goToCentre;
            break;

        case intersectionStates::goToCentre:
            robot_speed.w = pid_imu.step(imu_class->planar_integrator_.getYaw(), 0.01);
            robot_speed.v = pid_moveToTargetAhead.step( 0.43 - coordinates.x, 0.01);
            if (coordinates.x > 0.4)
                current_intersection_state = intersectionStates::spin;
            break;

        case intersectionStates::spin:
            robot_speed.v = 0;
            //call function where to go
            int where = 1;
            switch (where)
            {
                case 0:
                    robot_speed.w = pid_imu.step( imu_class->planar_integrator_.getYaw() + M_PI/2, 0.01); //left
                    if (imu_class->planar_integrator_.getYaw() > -M_PI/2 - 0.0698131701 && imu_class->planar_integrator_.getYaw() < -M_PI/2 + 0.0698131701 ) // bulharka na urcenni pm 5 stupnu
                        current_intersection_state = intersectionStates::ImuReset;
                    break;
                case 1:
                    robot_speed.w = pid_imu.step( imu_class->planar_integrator_.getYaw() - M_PI/2, 0.01); //right
                    if (imu_class->planar_integrator_.getYaw() < M_PI/2 + 0.0698131701 && imu_class->planar_integrator_.getYaw() > M_PI/2 - 0.0698131701 ) // bulharka na urcenni pm 5 stupnu
                        current_intersection_state = intersectionStates::ImuReset;
                    break;
                case 2:
                    current_intersection_state = intersectionStates::ImuReset; //straight
                    break;

            }
            robot_speed.w = pid_imu.step( imu_class->planar_integrator_.getYaw() - M_PI/2, 0.01);
            if (imu_class->planar_integrator_.getYaw() < M_PI/2 + 0.0698131701 && imu_class->planar_integrator_.getYaw() > M_PI/2 - 0.0698131701 ) // bulharka na urcenni pm 5 stupnu
            break;
        }
    }

    void MazeNode::line_select() {
        if (std::abs(lidar_class->get_error_angle(line)) > 0.3) {
            if (line == corridorLeft) {
                line = corridorRight;
                //std::cout << "prava strana " << std::endl;
            }
            else {
                line = corridorLeft;
                //std::cout << "leva strana " << std::endl;
            }
        }
    }




}
