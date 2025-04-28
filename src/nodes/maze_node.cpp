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
        current_state = next_state(current_state);

        if (current_state == states::calibration)
            std::cout << "calibration" << std::endl;
        if (current_state == states::intersection)
            std::cout << "intersection" << std::endl;
        if (current_state == states::corridorFollowing)
            std::cout << "corridorFollowing" << std::endl;
        if (current_state == states::resetImu)
            std::cout << "resetImu" << std::endl;
        if (current_state == states::turning)
            std::cout << "turning" << std::endl;



        switch (current_state)
        {
            case states::calibration:
                state_calibration();
                break;

            case states::corridorFollowing:
                state_corridor();
                break;

            case states::turning:
                state_turning();
                break;

            case states::resetImu:
                state_resetImu();
                break;

            case states::intersection:
                state_intersection();
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
                return states::corridorFollowing;
            return states::calibration;
        }
        else if (currentState == states::corridorFollowing) {
            if (tmp == middleX)
                return states::intersection;
            else if (tmp == leftTurn)
            {
                turning_spin_ = algorithms::spin::left;
                return states::turning;
            }
            else if (tmp == rightTurn)
            {
                turning_spin_ = algorithms::spin::right;
                return states::turning;
            }
            else if (tmp == blindEnd)
            {
                turning_spin_ = algorithms::spin::around;
                return states::turning;
            }
            else if (tmp == T)
            {
                turning_spin_ = camera_class->aruco_detector.spin_planner_.get_spin(lidar_class->intersection_scan());
                return states::turning;
            }
            return states::corridorFollowing;
        }
        else if (currentState == states::turning) {
            if (current_turning_state == turningStates::turningFinished) {
                current_turning_state = moveToTargetAhead;
                return states::resetImu;
            }
            return states::turning;
        }

        else if (currentState == states::resetImu) {
            if (current_imu_state == imuStates::ImuFinished)
            {
                current_imu_state = imuStates::getLine;
                return states::corridorFollowing;
            }
            return states::resetImu;
        }

        else if (currentState == states::intersection)
        {
            if (current_intersection_state == intersectionStates::intersectionFinished)
            {
                current_intersection_state = intersectionStates::resetCoordinates;
                return states::resetImu;
            }
            return states::intersection;
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

        if (0.02 > abs(lidar_class->get_error_angle(line_toFollow))) {
            imu_class->planar_integrator_.reset();

        }
    }

    void MazeNode::state_turning()
    {
        switch (current_turning_state)
        {
        case turningStates::moveToTargetAhead: //Go to the front barier
            robot_speed.w = pid_imu.step(imu_class->planar_integrator_.getYaw(), 0.01);
            robot_speed.v = pid_moveToTargetAhead.step( lidar_class->from_straight()-0.18, 0.01);
            //std::cout << lidar_class->from_straight() << std::endl;
            if (lidar_class->from_straight() < 0.21)
                current_turning_state = turningStates::turnings;
            break;

            case turningStates::turnings:
                switch (turning_spin_)
                {
                    case algorithms::left:
                        robot_speed.v = 0;
                        robot_speed.w = pid_imu.step( imu_class->planar_integrator_.getYaw() + M_PI/2, 0.01);
                        if (imu_class->planar_integrator_.getYaw() > -M_PI/2 - 0.0698131701 && imu_class->planar_integrator_.getYaw() < -M_PI/2 + 0.0698131701 ) // bulharka na urcenni pm 5 stupnu
                            current_turning_state = turningStates::turningFinished;
                        break;

                    case algorithms::right:
                        robot_speed.v = 0;
                        robot_speed.w = pid_imu.step( imu_class->planar_integrator_.getYaw() - M_PI/2, 0.01);
                        if (imu_class->planar_integrator_.getYaw() < M_PI/2 + 0.0698131701 && imu_class->planar_integrator_.getYaw() > M_PI/2 - 0.0698131701 ) // bulharka na urcenni pm 5 stupnu
                            current_turning_state = turningStates::turningFinished;
                        break;

                    case algorithms::around:
                        robot_speed.v = 0;
                        robot_speed.w = pid_imu.step( imu_class->planar_integrator_.getYaw() - M_PI, 0.01);
                        if (imu_class->planar_integrator_.getYaw() < M_PI + 0.0698131701 && imu_class->planar_integrator_.getYaw() > M_PI - 0.0698131701 ) // bulharka na urcenni pm 5 stupnu
                            current_turning_state = turningStates::turningFinished;
                        break;
                    default:
                        current_turning_state = turningStates::turningFinished;
                        break;
                }
                break;

        case turningStates::turningFinished:
            robot_speed.w = 0;
            robot_speed.v = 0;
            break;

        default:
            break;
        }
    }

    void MazeNode::state_intersection()
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
            std::cout << coordinates.x << std::endl;
            if (coordinates.x > 0.4){
                intersection_spin_ = camera_class->aruco_detector.spin_planner_.get_spin(lidar_class->intersection_scan());
                if (intersection_spin_ == algorithms::left)
                    std::cout << "left" << std::endl;
                if (intersection_spin_ == algorithms::right)
                    std::cout << "right" << std::endl;
                if (intersection_spin_ == algorithms::straight)
                    std::cout << "straight" << std::endl;
                current_intersection_state = intersectionStates::spin;
            }
            break;

        case intersectionStates::spin:
            robot_speed.v = 0;
            switch (intersection_spin_)
            {
                case algorithms::left:
                    robot_speed.w = pid_imu.step( imu_class->planar_integrator_.getYaw() + M_PI/2, 0.01); //left
                    if (imu_class->planar_integrator_.getYaw() > -M_PI/2 - 0.0698131701 && imu_class->planar_integrator_.getYaw() < -M_PI/2 + 0.0698131701 ) // bulharka na urcenni pm 5 stupnu
                        current_intersection_state = intersectionStates::intersectionFinished;
                    break;
                case algorithms::right:
                    robot_speed.w = pid_imu.step( imu_class->planar_integrator_.getYaw() - M_PI/2, 0.01); //right
                    if (imu_class->planar_integrator_.getYaw() < M_PI/2 + 0.0698131701 && imu_class->planar_integrator_.getYaw() > M_PI/2 - 0.0698131701 ) // bulharka na urcenni pm 5 stupnu
                        current_intersection_state = intersectionStates::intersectionFinished;
                    break;
                case algorithms::straight:
                    current_intersection_state = intersectionStates::intersectionFinished; //straight
                    break;

                default: ;
            }
        default: ;
        }
    }

    void MazeNode::state_resetImu()
    {
    switch (current_imu_state)
    {
        case getLine:
            line_parallel_select();
            current_imu_state = imuStates::centre;
            break;

    case centre:
        imu_class->planar_integrator_.reset_imu_angle(lidar_class->get_error_angle(line_toCentre));
            /*
            robot_speed.v = 0;
            robot_speed.w = pid_coridor_angle.step(lidar_class->get_error_angle(line_toCentre), 0.01);
            if (0.01 > abs(lidar_class->get_error_angle(line_toCentre)))
            {
                imu_class->planar_integrator_.reset();
                current_imu_state = imuStates::ImuFinished;
            }
            */
        reset_coordinates();
        current_imu_state = imuStates::littleGo;


        case littleGo:
            //robot_speed.w = pid_imu.step(imu_class->planar_integrator_.getYaw(), 0.01);
            robot_speed.w = 0;
            robot_speed.v = pid_moveToTargetAhead.step( 0.18 - coordinates.x, 0.01);
            if (coordinates.x > 0.1){
                current_imu_state = imuStates::ImuFinished;
            }
        break;;

        case ImuFinished:
            break;
    }
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




}
