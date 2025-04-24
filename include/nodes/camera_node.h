//
// Created by martin on 19.04.25.
//

#ifndef CAMERA_NODE_H
#define CAMERA_NODE_H
//
// Created by student on 3.3.25.
//

#include <rclcpp/rclcpp.hpp>


namespace nodes {
    class CameraNode : public rclcpp::Node {
    public:
        // Constructor
        CameraNode();
        // Destructor (default)
        ~CameraNode() override = default;
    private:

    };
}
#endif //CAMERA_NODE_H
