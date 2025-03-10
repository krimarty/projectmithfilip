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

    std::pair<float, float> total = std::make_pair(0, 0);
    std::pair<float, float> myPair = std::make_pair(-3, 0.00);
    std::pair<int, int> p = kinematics_object.Inverse_kinematics(myPair);

    while (rclcpp::ok())
    {
        //std::cout << example_class1->get_button_pressed() << std::endl;
        /*
        switch (example_class1->get_button_pressed())
        {
            case 0:
                example_class1->publish_message(0);
                    break;
            case 1:
                example_class1->publish_message(4);
                    break;
            case 2:
                example_class1->publish_message(8);
                    break;
            default:
                break;

        }
        */



        motor_class->publish_message(p.first, p.second);

        std::pair<int, int> encoders;
        encoders = encoder_class->get_values();
        std::pair<float, float> souradnice;
        souradnice  = kinematics_object.Forward_odometry(encoders);

        total.first = total.first + souradnice.first;
        total.second = total.second + souradnice.second;
        std::cout << total.first << ", " << total.second << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
