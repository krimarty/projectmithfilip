#include <nodes/lidar_node.h>
#include <rclcpp/rclcpp.hpp>
#include "nodes/joystick_node.h"
#include "nodes/line_node.h"
#include "algorithms/pid.h"
#include "nodes/maze_node.h"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    // Create an executor (for handling multiple nodes)
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    // Create instances of RosExampleClass using the existing nodes
    auto joystick_class = std::make_shared<nodes::JoystickNode>();

    auto line_class = std::make_shared<nodes::LineNode>();


    auto maze_class = std::make_shared<nodes::MazeNode>();
    auto imu = std::make_shared<nodes::ImuNode>();

    // Add nodes to the executor
    executor->add_node(joystick_class);
    executor->add_node(line_class);



    executor->add_node(maze_class);
    //executor->add_node(imu);
    executor->add_node(maze_class->imu_class);
    executor->add_node(maze_class->lidar_class);
    executor->add_node(maze_class->motor_class);
    executor->add_node(maze_class->io_class);
    executor->add_node(maze_class->camera_class);
    executor->add_node(maze_class->encoder_class);


    // Run the executor (handles callbacks for both nodes)
    auto executor_thread = std::thread([&executor]() { executor->spin(); });

    while (rclcpp::ok())
    {
        //coridor_class->corridor_routine();
        maze_class->maze_routine();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
