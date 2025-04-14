//
// Created by student on 14.4.25.
//

#include "nodes/imu_node.h"

namespace nodes{
    ImuNode::ImuNode() : Node("imu_node"){

        imu_subscriber_ = create_subscription<sensor_msgs::msg::Imu>(
           "/bpc_prp_robot/imu", 1, std::bind(&ImuNode::on_imu_msg, this, std::placeholders::_1));
    }

    void ImuNode::on_imu_msg(const sensor_msgs::msg::Imu::SharedPtr msg){
        auto time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = lastTime - time;
        lastTime = time;

        if (gyro_calibration_samples_.size() < 300)
        {
            float tmp = msg->angular_velocity.z;
            gyro_calibration_samples_.push_back(tmp);

            if (gyro_calibration_samples_.size() == 300)
            {
                planar_integrator_.setCalibration(gyro_calibration_samples_);
                calibrated_ = true;
            }
        }
        else
        {
            planar_integrator_.update(msg->angular_velocity.z, elapsed.count());
            std::cout << planar_integrator_.getYaw() << std::endl;
        }


    }


}

