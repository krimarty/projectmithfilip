#include <nodes/lidar_node.h>
#include <rclcpp/rclcpp.hpp>
#include "nodes/io_node.h"
#include "nodes/encoder_node.h"
#include "nodes/joystick_node.h"
#include "nodes/line_node.h"
#include "algorithms/pid.h"
#include "nodes/coridor_node.h"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    // Create an executor (for handling multiple nodes)
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    // Create instances of RosExampleClass using the existing nodes
    auto example_class1 = std::make_shared<nodes::IoNode>();
    auto joystick_class = std::make_shared<nodes::JoystickNode>();

    auto encoder_class = std::make_shared<nodes::EncoderNode>();
    auto line_class = std::make_shared<nodes::LineNode>();

    auto coridor_class = std::make_shared<nodes::CorridorNode>();

    // Add nodes to the executor
    executor->add_node(example_class1);
    executor->add_node(joystick_class);
    executor->add_node(encoder_class);
    executor->add_node(line_class);

    executor->add_node(coridor_class);
    executor->add_node(coridor_class->imu_class);
    executor->add_node(coridor_class->lidar_class);
    executor->add_node(coridor_class->motor_class);


    // Run the executor (handles callbacks for both nodes)
    auto executor_thread = std::thread([&executor]() { executor->spin(); });

    while (rclcpp::ok())
    {
        coridor_class->corridor_routine();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
