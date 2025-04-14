//
// Created by student on 14.4.25.
//

#ifndef IMU_NODE_H
#define IMU_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <algorithms/planar_imu_integrator.h>
#include <chrono>

namespace nodes {

    enum class ImuNodeMode {
        CALIBRATE,
        INTEGRATE,
    };

    class ImuNode : public rclcpp::Node
    {
    public:
        ImuNode();
        ~ImuNode() override = default;

        // Set the IMU Mode
        void setMode(const ImuNodeMode setMode);

        // Get the current IMU Mode
        ImuNodeMode getMode();

        // Get the results after Integration
        auto getIntegratedResults();

        // Reset the class
        void reset_imu();
        algorithms::PlanarImuIntegrator planar_integrator_;
        bool calibrated_ = false;
    private:

        void calibrate();
        void integrate();

        ImuNodeMode mode = ImuNodeMode::INTEGRATE;

        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;


        std::vector<float> gyro_calibration_samples_;



        std::chrono::time_point<std::chrono::system_clock> lastTime{};



        void on_imu_msg(const sensor_msgs::msg::Imu::SharedPtr msg);
    };
}


#endif //IMU_NODE_H
