#include <rclcpp/rclcpp.hpp>
#include "RosExampleClass.h"
#include "nodes/io_node.h"
#include "nodes/motor_node.h"
#include "nodes/encoder_node.h"
#include "algorithms/kinematics_algorithms.h"
#include "nodes/joystick_node.h"
#include "nodes/line_loop.h"

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
    //auto line_class = std::make_shared<nodes::LineNode>();
    auto line_loop = std::make_shared<nodes::LineLoop>();

    algorithms::KinematicsAlgorithms kinematics_object;



    // Add nodes to the executor
    executor->add_node(example_class1);
    executor->add_node(joystick_class);
    //executor->add_node(motor_class);
    //executor->add_node(encoder_class);
    //executor->add_node(line_class);
    executor->add_node(line_loop);
    //executor->add_node(node2);

    // Run the executor (handles callbacks for both nodes)
    auto executor_thread = std::thread([&executor]() { executor->spin(); });

    Coordinates coordinates{0 ,0};
    WheelSpeed wheel_speed{};
    RobotSpeed robot_speed{10, 0.5};
    Encoders encoders{};
    Encoders tmp_encoders{};
    Pose pose{};
    int blink = 0;



    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    tmp_encoders.l = encoder_class->get_left_value();
    tmp_encoders.r = encoder_class->get_right_value();

    wheel_speed = algorithms::KinematicsAlgorithms::Inverse_kinematics(robot_speed);

    while (rclcpp::ok())
    {
        //blikani
        example_class1->publish_message(blink);
        if (blink == 0) {
            blink = 1;
        } else {
            blink = 0;
        }

        auto ovladac = joystick_class->get_v_();
        auto ovladac1 = joystick_class->get_w_();

        //if (ovladac1 != 0 || ovladac != 0)
        //{
        //    robot_speed.v = ovladac;
        //    robot_speed.w = ovladac1;
        //    wheel_speed = algorithms::KinematicsAlgorithms::Inverse_kinematics(robot_speed);
        //    motor_class->publish_motorSpeed(wheel_speed.l, wheel_speed.r);

       // }
       // else
       // {
            line_loop->line_loop_timer_callback();
      //  }


        //Slimovina
        //std:: cout << line_class->get_continuous_line_pose() << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
