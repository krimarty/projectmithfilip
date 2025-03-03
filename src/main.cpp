#include <rclcpp/rclcpp.hpp>
#include "RosExampleClass.h"
#include "nodes/io_node.h"

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

    // Add nodes to the executor
    executor->add_node(example_class1);
    //executor->add_node(node2);

    // Run the executor (handles callbacks for both nodes)
    auto executor_thread = std::thread([&executor]() { executor->spin(); });
    while (rclcpp::ok())
    {
        std::cout << example_class1->get_button_pressed() << std::endl;
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
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
