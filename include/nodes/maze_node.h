//
// Created by martin on 16.04.25.
//

#ifndef MAZE_NODE_H
#define MAZE_NODE_H

#include <rclcpp/rclcpp.hpp>
#include "algorithms/pid.h"
#include "algorithms/kinematics_algorithms.h"
#include "nodes/imu_node.h"
#include <nodes/lidar_node.h>
#include "nodes/motor_node.h"
#include "nodes/encoder_node.h"
#include "nodes/io_node.h"
#include "nodes/camera_node.h"


namespace nodes {

    enum states
    {
        calibration,
        corridorFollowing,
        moveToTarget,
        resetCoordinates,
        moveToCenterCoordinates,
        getSpin,
        turnLeft,
        turnRight,
        around,
        littleGo,
    };

    class MazeNode : public rclcpp::Node {
    public:
        // Constructor
        MazeNode();
        // Destructor (default)
        ~MazeNode() override = default;


        void maze_routine();

        void state_calibration();
        void state_corridor();
        void state_moveto_target();
        void state_reset_coordinates();
        void state_moveto_center();
        void state_getSpin();
        void state_left();
        void state_right();
        void state_around();
        void state_littleGo();

        void reset_imu();

        void update_coordinates()
        {
            new_encoders.l = encoder_class->get_left_value() - old_encoders.l;
            new_encoders.r = encoder_class->get_right_value() - old_encoders.r;
            old_encoders.l = encoder_class->get_left_value();
            old_encoders.r = encoder_class->get_right_value();

            Coordinates tmp = algorithms::KinematicsAlgorithms::Forward_odometry(new_encoders);
            coordinates.x = coordinates.x + tmp.x;
            coordinates.y = coordinates.y + tmp.y;
        }

        void reset_coordinates()
        {
            old_encoders.l = encoder_class->get_left_value();
            old_encoders.r = encoder_class->get_right_value();
            coordinates.x = 0;
            coordinates.y = 0;
        }



        std::shared_ptr<nodes::LidarNode> lidar_class;
        std::shared_ptr<nodes::EncoderNode> encoder_class;
        std::shared_ptr<nodes::ImuNode> imu_class;
        std::shared_ptr<nodes::MotorNode> motor_class;
        std::shared_ptr<nodes::IoNode> io_class;
        std::shared_ptr<nodes::CameraNode> camera_class;


    private:
        states current_state = states::calibration;

        algorithms::spin turning_spin_;
        algorithms::spin intersection_spin_;

        algorithms::Pid pid_coridor_center;
        algorithms::Pid pid_coridor_angle;
        algorithms::Pid pid_moveToTargetAhead;
        algorithms::Pid pid_imu;

        WheelSpeed wheel_speed;
        RobotSpeed robot_speed;
        Encoders old_encoders{0};
        Encoders new_encoders{0};
        Coordinates coordinates{};
        states next_state(states currentState);
        void line_select();
        void line_parallel_select();


        mode line_toFollow = mode::leftFront;
        mode line_toCentre = mode::rightFront;

    };

}

#endif //MAZE_NODE_H
