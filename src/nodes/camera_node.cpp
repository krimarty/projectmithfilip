//
// Created by student on 24.4.25.
//
#include "nodes/camera_node.h"

namespace nodes
{
    CameraNode::CameraNode() : Node("camera_node") {
        image_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
        "/bpc_prp_robot/camera", 1, std::bind(&CameraNode::on_image_msg, this, std::placeholders::_1));
    }


    void CameraNode::on_image_msg(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        try
        {
            if (msg->encoding != "bgr8") {
                RCLCPP_WARN(this->get_logger(), "Unsupported encoding: %s. Expected 'bgr8'", msg->encoding.c_str());
                return;
            }

            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
            const cv::Mat& frame = cv_ptr->image;
            aruco_detector.detect(frame);

            cv::imshow("Robot Camera Feed", frame);
            cv::waitKey(1);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }
}

