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
       pid_coridor_angle(1.0, 0.002, 0.1),
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
        //current_state = next_state(current_state);
        states tmp_states = next_state(current_state);

        if (tmp_states != current_state)
        {
            current_state = tmp_states;
            if (current_state == states::calibration)
                std::cout << "calibration" << std::endl;
            if (current_state == states::corridorFollowing)
                std::cout << "corridorFollowing" << std::endl;
            if (current_state == states::moveToTarget)
                std::cout << "moveToTarget" << std::endl;
            if (current_state == states::resetCoordinates)
                std::cout << "resetCoordinates" << std::endl;
            if (current_state == states::moveToCenterCoordinates)
                std::cout << "moveToCenterCoordinates" << std::endl;
            if (current_state == states::getSpin)
                std::cout << "getSpin" << std::endl;
            if (current_state == states::turnLeft)
                std::cout << "turnLeft" << std::endl;
            if (current_state == states::turnRight)
                std::cout << "turnRight" << std::endl;
            if (current_state == states::around)
                std::cout << "around" << std::endl;
            if (current_state == states::littleGo)
                std::cout << "littleGo" << std::endl;
        }
        current_state = tmp_states;


        switch (current_state)
        {
            case states::calibration:
                state_calibration();
                break;

            case states::corridorFollowing:
                state_corridor();
                break;

            case states::moveToTarget:
                    state_moveto_target();
                    break;

            case states::resetCoordinates:
                state_reset_coordinates();
                break;

            case states::moveToCenterCoordinates:
                state_moveto_center();
                break;

            case states::getSpin:
                state_getSpin();
                break;

            case states::around:
                state_around();
                break;

            case states::turnLeft:
                state_left();
                break;

            case states::turnRight:
                state_right();
                break;

            case states::littleGo:
                state_littleGo();
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
            reset_coordinates();
            if (imu_class->calibrated_)
                return states::corridorFollowing;
            return states::calibration;
        }

        if (currentState == states::corridorFollowing)
        {
            if (tmp == middleX)
            {
                reset_coordinates();
                return states::resetCoordinates;
            }
            if (tmp == leftTurn) return states::moveToTarget;
            if (tmp == rightTurn) return states::moveToTarget;
            if (tmp == blindEnd) return states::moveToTarget;
            if (tmp == T) return states::moveToTarget;
            return states::corridorFollowing;
        }

        if (currentState == states::moveToTarget)
        {

            if (lidar_class->from_straight() > 0.8) return states::corridorFollowing;

            freeCorridor scan = lidar_class->intersection_scan();
            if (lidar_class->from_straight() < 0.21)
            {
                if (scan.left == true && scan.right == true) return states::getSpin;
                if (scan.left == false && scan.right == true) return states::turnRight;
                if (scan.left == true && scan.right == false) return states::turnLeft;
                if (scan.left == false && scan.right == false) return states::around;
            }
            return states::moveToTarget;
        }

        if (currentState == states::resetCoordinates) {
            if (lidar_class->is_new_cell())
            {
                reset_coordinates();
                return states::moveToCenterCoordinates;
            }
            if (coordinates.x > 0.3) return states::corridorFollowing;

            return states::resetCoordinates;
        }

        if (currentState == states::moveToCenterCoordinates)
        {
            freeCorridor scan = lidar_class->intersection_scan();
            if (lidar_class->from_straight() < 0.35) return states::moveToTarget;
            if (coordinates.x > 0.17)
            {
                if (scan.left == false && scan.right == false && scan.front == true) return states::corridorFollowing;
                return getSpin;
            }
            return states::moveToCenterCoordinates;
        }

        if (currentState == states::getSpin)
        {
            algorithms::spin spin = camera_class->aruco_detector.spin_planner_.get_spin(lidar_class->intersection_scan());
            if (spin == algorithms::spin::left) return states::turnLeft;
            if (spin == algorithms::spin::right) return states::turnRight;
            if (spin == algorithms::spin::straight)
            {
                reset_imu();
                return states::littleGo;
            }
            return states::corridorFollowing;//majbe
        }

        if (currentState == states::turnLeft)
        {
            if (imu_class->planar_integrator_.getYaw() > -M_PI/2 - 0.0698131701 && imu_class->planar_integrator_.getYaw() < -M_PI/2 + 0.0698131701 )
            {
                reset_imu();
                return states::littleGo;
            }
            return states::turnLeft;
        }

        if (currentState == states::turnRight)
        {
            if (imu_class->planar_integrator_.getYaw() < M_PI/2 + 0.0698131701 && imu_class->planar_integrator_.getYaw() > M_PI/2 - 0.0698131701 )
            {
                reset_imu();
                return states::littleGo;
            }
            return states::turnRight;
        }

        if (currentState == states::around)
        {
            if  (robot_speed.w < 0.05)
            {
                reset_imu();
                return states::littleGo;
            }
            return states::around;
        }

        if (currentState == states::littleGo)
        {
            if (coordinates.x > 0.1) return states::corridorFollowing;
            return states::littleGo;
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
            //imu_class->planar_integrator_.reset_imu_angle(lidar_class->get_error_angle(line_toCentre));
            robot_speed.w = pid_coridor_angle.step(lidar_class->get_error_angle(line_toFollow), 0.01);
            pid_coridor_center.step(lidar_class->get_error_distance(line_toFollow), 0.01);
            //std::cout << "Uhluju" << std::endl;
        }
        else
        {
            pid_coridor_angle.step(lidar_class->get_error_angle(line_toFollow), 0.01);
            robot_speed.w = pid_coridor_center.step(lidar_class->get_error_distance(line_toFollow), 0.01);
            //std::cout << "Centruju" << std::endl;
        }

        robot_speed.v = 0.1;

        if (0.01 > abs(lidar_class->get_error_angle(line_toFollow)))
            imu_class->planar_integrator_.reset();
    }

    void MazeNode::state_moveto_target()
    {
        //std::cout << "to target: " << lidar_class->from_straight() << std::endl;
        robot_speed.w = pid_imu.step(imu_class->planar_integrator_.getYaw(), 0.01);
        robot_speed.v = pid_moveToTargetAhead.step( lidar_class->from_straight()-0.18, 0.01);
    }

    void MazeNode::state_reset_coordinates()
    {
        robot_speed.v = 0.1;
        robot_speed.w = pid_imu.step(imu_class->planar_integrator_.getYaw(), 0.01);
    }

    void MazeNode::state_moveto_center()
    {
        robot_speed.v = pid_moveToTargetAhead.step( 0.2 - coordinates.x, 0.01);
        robot_speed.w = pid_imu.step(imu_class->planar_integrator_.getYaw(), 0.01);
    }

    void MazeNode::state_getSpin()
    {
        robot_speed.v = 0;
        robot_speed.w = 0;
    }

    void MazeNode::state_around()
    {
        robot_speed.v = 0;
        robot_speed.w = pid_imu.step( imu_class->planar_integrator_.getYaw() + (M_PI - 0.34906585), 0.01);
    }

    void MazeNode::state_left()
    {
        robot_speed.v = 0;
        robot_speed.w = pid_imu.step( imu_class->planar_integrator_.getYaw() + M_PI/2, 0.01);
    }

    void MazeNode::state_right()
    {
        robot_speed.v = 0;
        robot_speed.w = pid_imu.step( imu_class->planar_integrator_.getYaw() - M_PI/2, 0.01);
    }

    void MazeNode::state_littleGo()
    {
        robot_speed.w = pid_imu.step(imu_class->planar_integrator_.getYaw(), 0.01);
        robot_speed.v = pid_moveToTargetAhead.step( 0.14 - coordinates.x, 0.01);
    }



    void MazeNode::line_parallel_select()
    {
        mode tmp = mode::corridorLeft;
        if (std::abs(lidar_class->get_error_angle(tmp)) > std::abs(lidar_class->get_error_angle(mode::corridorRight)))
            tmp = mode::corridorRight;
        if (std::abs(lidar_class->get_error_angle(tmp)) > std::abs(lidar_class->get_error_angle(mode::left)))
            tmp = mode::left;
        if (std::abs(lidar_class->get_error_angle(tmp)) > std::abs(lidar_class->get_error_angle(mode::right)))
            tmp = mode::right;
        if (std::abs(lidar_class->get_error_angle(tmp)) > std::abs(lidar_class->get_error_angle(mode::leftBack)))
            tmp = mode::leftBack;
        if (std::abs(lidar_class->get_error_angle(tmp)) > std::abs(lidar_class->get_error_angle(mode::rightBack)))
            tmp = mode::rightBack;
        if (std::abs(lidar_class->get_error_angle(tmp)) > std::abs(lidar_class->get_error_angle(mode::leftFront)))
            tmp = mode::leftFront;
        if (std::abs(lidar_class->get_error_angle(tmp)) > std::abs(lidar_class->get_error_angle(mode::rightFront)))
            tmp = mode::rightFront;

        line_toCentre = tmp;
    }


    void MazeNode::line_select() {
        if (std::abs(lidar_class->get_error_angle(line_toFollow)) > 0.3) {
            if (line_toFollow == corridorLeft) {
                line_toFollow = corridorRight;
            }
            else {
                line_toFollow = corridorLeft;
            }
        }
    }

    void MazeNode::reset_imu()
    {
        line_parallel_select();
        std::cout << "uhel: " <<std::atan(lidar_class->get_error_angle(line_toCentre)*180/M_PI) << std::endl;
        imu_class->planar_integrator_.reset_imu_angle(lidar_class->get_error_angle(line_toCentre));
        reset_coordinates();
    }





}
