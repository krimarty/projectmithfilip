//
// Created by student on 24.4.25.
//
#include "nodes/camera_node.h"

namespace nodes
{
    CameraNode::CameraNode() : Node("camera_node") {
        image_subscriber_ = create_subscription<sensor_msgs::msg::CompressedImage>(
        "/bpc_prp_robot/camera/compressed", 10, std::bind(&CameraNode::on_image_msg, this, std::placeholders::_1));
    }


    void CameraNode::on_image_msg(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        try
        {
            // Převod std::vector<uint8_t> na cv::Mat
            cv::Mat compressed_data(1, static_cast<int>(msg->data.size()), CV_8UC1, const_cast<uchar*>(msg->data.data()));

            // Dekódování JPEG/PNG do BGR obrázku
            cv::Mat frame = cv::imdecode(compressed_data, cv::IMREAD_COLOR);

            if (frame.empty()) {
                RCLCPP_WARN(this->get_logger(), "Decoded frame is empty.");
                return;
            }

            aruco_detector.detect(frame);

            cv::imshow("Robot Camera Feed", frame);
            cv::waitKey(1);
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception while decoding image: %s", e.what());
        }
    }

}

