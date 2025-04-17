//
// Created by martin on 16.04.25.
//

#ifndef CORIDOR_NODE_H
#define CORIDOR_NODE_H

#include <rclcpp/rclcpp.hpp>
#include "algorithms/pid.h"
#include "algorithms/kinematics_algorithms.h"
#include "nodes/imu_node.h"
#include <nodes/lidar_node.h>
#include "nodes/motor_node.h"
#include "nodes/io_node.h"


namespace nodes {

    enum states
    {
        calibration,
        corridor_following,
        center,
        turningLeft,
        turningRight,
    };

    enum turning_states
    {
        moveToTargetAhead,
        turning,
    };

    class CorridorNode : public rclcpp::Node {
    public:
        // Constructor
        CorridorNode();
        // Destructor (default)
        ~CorridorNode() override = default;


        void corridor_routine();

        void state_calibration();
        void state_corridor();
        void state_left();
        void state_right();
        void state_center();

        std::shared_ptr<nodes::LidarNode> lidar_class;
        std::shared_ptr<nodes::ImuNode> imu_class;
        std::shared_ptr<nodes::MotorNode> motor_class;
        std::shared_ptr<nodes::IoNode> io_class;

    private:
        states current_state = states::calibration;
        turning_states current_turning_state = turning_states::moveToTargetAhead;
        algorithms::Pid pid_coridor_center;
        algorithms::Pid pid_coridor_angle;
        algorithms::Pid pid_moveToTargetAhead;
        algorithms::Pid pid_imu;

        WheelSpeed wheel_speed;
        RobotSpeed robot_speed;

        states next_state(states currentState);
        mode line = mode::leftFront;


    };
}

#endif //CORIDOR_NODE_H
