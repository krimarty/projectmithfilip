#include <nodes/lidar_node.h>
#include <rclcpp/rclcpp.hpp>
#include "RosExampleClass.h"
#include "nodes/io_node.h"
#include "nodes/motor_node.h"
#include "nodes/encoder_node.h"
#include "algorithms/kinematics_algorithms.h"
#include "nodes/joystick_node.h"
#include "nodes/line_node.h"
#include "algorithms/pid.h"
#include "nodes/imu_node.h"

enum states
{
    calibration,
    corridor_following,
    intersection,
    turning,

};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    // Create an executor (for handling multiple nodes)
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    // Create multiple nodes
    //auto node1 = std::make_shared<rclcpp::Node>("node1");
    //auto node2 = std::make_shared<rclcpp::Node>("node2");

    // Create instances of RosExampleClass using the existing nodes
    auto example_class1 = std::make_shared<nodes::IoNode>();
    auto joystick_class = std::make_shared<nodes::JoystickNode>();
    //auto example_class2 = std::make_shared<RosExampleClass>(node2, "topic2", 2.0);
    auto motor_class = std::make_shared<nodes::MotorNode>();
    auto encoder_class = std::make_shared<nodes::EncoderNode>();
    auto line_class = std::make_shared<nodes::LineNode>();
    auto lidar_class = std::make_shared<nodes::LidarNode>();
    auto imu_class = std::make_shared<nodes::ImuNode>();


    algorithms::Pid pid_coridor(0.8, 0.004 ,0);
    algorithms::Pid pid_imu(0.5, 0.004 ,0);
    algorithms::KinematicsAlgorithms kinematics_object;



    // Add nodes to the executor
    executor->add_node(example_class1);
    executor->add_node(joystick_class);
    executor->add_node(motor_class);
    executor->add_node(encoder_class);
    executor->add_node(line_class);
    executor->add_node(lidar_class);
    executor->add_node(imu_class);
    //executor->add_node(node2);

    // Run the executor (handles callbacks for both nodes)
    auto executor_thread = std::thread([&executor]() { executor->spin(); });

    Coordinates coordinates{0 ,0};
    WheelSpeed wheel_speed{};
    RobotSpeed robot_speed{0, 0
    };
    Encoders encoders{};
    Encoders tmp_encoders{};
    Pose pose{};
    int blink = 0;



    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    tmp_encoders.l = encoder_class->get_left_value();
    tmp_encoders.r = encoder_class->get_right_value();

    wheel_speed = algorithms::KinematicsAlgorithms::Inverse_kinematics(robot_speed);


    states current_state = states::calibration;

    while (rclcpp::ok())
    {
/*
        // Toceni s motory
        //robot_speed.v = joystick_class->get_v_();
        //robot_speed.w = joystick_class->get_w_();
        //wheel_speed = algorithms::KinematicsAlgorithms::Inverse_kinematics(robot_speed);
        //motor_class->publish_motorSpeed(wheel_speed.l, wheel_speed.r);

        // Ziskani diference z enkoderu
        encoders.l = encoder_class->get_left_value() - tmp_encoders.l;
        encoders.r = encoder_class->get_right_value() -tmp_encoders.r;
        tmp_encoders.l = encoder_class->get_left_value();
        tmp_encoders.r = encoder_class->get_right_value();

        // Vypocet nove pozy
        pose = algorithms::KinematicsAlgorithms::update_pose(pose, encoders);
        //std::cout << pose.x << " m, " << pose.y << " m, " << pose.theta << " rad" << std::endl;

        // Nahodne veci Filipa Slimy
        Coordinates tmp_coordinates = algorithms::KinematicsAlgorithms::Forward_odometry(encoders);
        coordinates.x = coordinates.x + tmp_coordinates.x;
        coordinates.y = coordinates.y + tmp_coordinates.y;
        //std::cout << tmp_coordinates.x << ", " << tmp_coordinates.y << std::endl;
        //std::cout << coordinates.x << ", " << coordinates.y << std::endl;


        /*
        // LINE FOLLOWING - BANG BANG I HIT THE GROUND, PRIORITNE JOY
        nodes::DiscreteLinePose tmp = line_class->get_discrete_line_pose();
        auto ovladac = joystick_class->get_v_();
        auto ovladac1 = joystick_class->get_w_();

        if (ovladac1 != 0 || ovladac != 0)
        {
            robot_speed.v = ovladac;
            robot_speed.w = ovladac1;
        }
        else
        {
            if (tmp == nodes::DiscreteLinePose::LineNone)
            {
                //std::cout << "DiscreteLinePose::LineNone" << std::endl;
                robot_speed.w = 0;
            }
            /*if (tmp == nodes::DiscreteLinePose::LineBoth)
                std::cout << "DiscreteLinePose::LineBoth" << std::endl;
            else if (tmp == nodes::DiscreteLinePose::LineOnLeft)
            {
                //std::cout << "DiscreteLinePose::LineOnLeft" << std::endl;
                robot_speed.w = 0.12;
            }
            else if (tmp == nodes::DiscreteLinePose::LineOnRight)
            {
                //std::cout << "DiscreteLinePose::LineOnRight" << std::endl;
                robot_speed.w = -0.12;
            }
            robot_speed.v = 0.025;
        }
        //std:: cout << line_class->get_continuous_line_pose() << std::endl;


        wheel_speed = algorithms::KinematicsAlgorithms::Inverse_kinematics(robot_speed);
        motor_class->publish_motorSpeed(wheel_speed.l, wheel_speed.r);
        */ // END OF LINE FOLLOWING
        //Slimovina

        //drzhubukrizu


        //std:: cout << line_class->get_continuous_line_pose() << std::endl;
        //std::this_thread::sleep_for(std::chrono::milliseconds(100));

        /*
        //LIDAR PID
        robot_speed.w = pid_coridor.step(lidar_class->get_result(), 0.01);
        std::cout << robot_speed.w << std::endl;
        //robot_speed.v = 0.05;
        robot_speed.v = 0.035;

        wheel_speed = algorithms::KinematicsAlgorithms::Inverse_kinematics(robot_speed);
        motor_class->publish_motorSpeed(wheel_speed.l, wheel_speed.r);

        */

        switch (current_state)
        {
            case states::calibration:
                if (imu_class->calibrated_)
                    current_state = states::corridor_following;
                break;
            case states::corridor_following:
                if (line_class->line_detected())
                {
                    pose.x = 0; pose.y = 0; pose.theta = 0;
                    tmp_encoders.l = encoder_class->get_left_value();
                    tmp_encoders.r = encoder_class->get_right_value();
                    current_state = states::intersection;
                }
                else
                {
                    robot_speed.w = pid_coridor.step(lidar_class->get_result(), 0.01);
                    //std::cout << robot_speed.w << std::endl;
                    robot_speed.v = 0.035;
                }

                break;
            case states::intersection:
                if (pose.x < 0.2)
                {
                    // Ziskani diference z enkoderu
                    encoders.l = encoder_class->get_left_value() - tmp_encoders.l;
                    encoders.r = encoder_class->get_right_value() -tmp_encoders.r;
                    tmp_encoders.l = encoder_class->get_left_value();
                    tmp_encoders.r = encoder_class->get_right_value();

                    // Vypocet nove pozy
                    pose = algorithms::KinematicsAlgorithms::update_pose(pose, encoders);

                    robot_speed.v = 0.02;
                    robot_speed.w = 0;
                }
                else if (pose.x > 0.2)
                {
                    robot_speed.v = 0;
                    robot_speed.w = 0;
                }
                //std::cout << pose.x << " m, " << pose.y << " m, " << pose.theta << " rad" << std::endl;


                break;
            case states::turning:
                /*
                yaw_error = imu_class->planar_integrator_.getYaw();
                robot_speed.w = pid_imu.step(yaw_error, 0.01);
                robot_speed.v = 0;
                */
                break;
        }



        wheel_speed = algorithms::KinematicsAlgorithms::Inverse_kinematics(robot_speed);
        //motor_class->publish_motorSpeed(wheel_speed.l, wheel_speed.r);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
