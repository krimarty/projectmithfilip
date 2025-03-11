#include <rclcpp/rclcpp.hpp>
#include "RosExampleClass.h"
#include "nodes/io_node.h"
#include "nodes/motor_node.h"
#include "nodes/encoder_node.h"
#include "algorithms/kinematics_algorithms.h"

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
    //auto example_class2 = std::make_shared<RosExampleClass>(node2, "topic2", 2.0);
    auto motor_class = std::make_shared<nodes::MotorNode>();
    auto encoder_class = std::make_shared<nodes::EncoderNode>();

    algorithms::KinematicsAlgorithms kinematics_object;



    // Add nodes to the executor
    executor->add_node(example_class1);
    executor->add_node(motor_class);
    executor->add_node(encoder_class);
    //executor->add_node(node2);

    // Run the executor (handles callbacks for both nodes)
    auto executor_thread = std::thread([&executor]() { executor->spin(); });

    Coordinates coordinates{0 ,0};
    WheelSpeed wheel_speed{};
    RobotSpeed robot_speed{10, 0.5};
    Encoders encoders{};
    Encoders tmp_encoders{};
    Pose pose{};



    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    tmp_encoders.l = encoder_class->get_left_value();
    tmp_encoders.r = encoder_class->get_right_value();

    wheel_speed = algorithms::KinematicsAlgorithms::Inverse_kinematics(robot_speed);

    while (rclcpp::ok())
    {
        motor_class->publish_motorSpeed(wheel_speed.l, wheel_speed.r);

        encoders.l = encoder_class->get_left_value() - tmp_encoders.l;
        encoders.r = encoder_class->get_right_value() -tmp_encoders.r;
        tmp_encoders.l = encoder_class->get_left_value();
        tmp_encoders.r = encoder_class->get_right_value();
        std::cout << encoders.l << ", " << encoders.r << std::endl;


        Coordinates tmp_coordinates = algorithms::KinematicsAlgorithms::Forward_odometry(encoders);
        //Encoders encoders_megatmp{576, 3*576};
        //Coordinates tmp_coordinates_megatmp= algorithms::KinematicsAlgorithms::Forward_odometry(encoders_megatmp);


        coordinates.x = coordinates.x + tmp_coordinates.x;
        coordinates.y = coordinates.y + tmp_coordinates.y;
        std::cout << tmp_coordinates.x << ", " << tmp_coordinates.y << std::endl;
        std::cout << coordinates.x << ", " << coordinates.y << std::endl;

        pose = algorithms::KinematicsAlgorithms::update_pose(pose, robot_speed, nodes::EncoderNode::T);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
