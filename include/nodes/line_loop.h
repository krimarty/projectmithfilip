//
// Created by student on 24.3.25.
//

#ifndef LINE_LOOP_H
#define LINE_LOOP_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <algorithms/pid.h>
#include <atomic>

#define L_MIN 30
#define L_MAX 920
#define R_MIN 30
#define R_MAX 980

#define LSM_A (-0.13)
#define LSM_B 0

namespace nodes {
    enum class DiscreteLinePose {
        LineOnLeft,
        LineOnRight,
        LineNone,
        LineBoth,
    };

    class LineLoop : public rclcpp::Node {
    public:

        LineLoop();

        ~LineLoop() override = default;

        // relative pose to line in meters
        float get_continuous_line_pose()
        {
            return estimate_continuous_line_pose(l_sensor, r_sensor);
        }

        DiscreteLinePose get_discrete_line_pose()
        {
            return estimate_discrete_line_pose(l_sensor, r_sensor);
        }

        void publish_motorSpeed(double l, double r);

        void line_loop_timer_callback();

    private:

        std::atomic<float> l_sensor;
        std::atomic<float> r_sensor;

        rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr line_sensors_subscriber_;
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motorSpeed_publisher_;
        std::shared_ptr<algorithms::Pid> pid_;

        void on_line_sensors_msg(std::shared_ptr<std_msgs::msg::UInt16MultiArray> msg)
        {
            const int tmp1 = msg->data[0];
            const int tmp2 = msg->data[1];
            l_sensor = (static_cast<float>(tmp1) - L_MIN) / (L_MAX - L_MIN);
            r_sensor = (static_cast<float>(tmp2) - R_MIN) / (R_MAX - R_MIN);
            //std::cout << "l_sensor: " << l_sensor << " r_sensor: " << r_sensor << msg->data[0] << std::endl;
        }

        float estimate_continuous_line_pose(float left_value, float right_value);

        DiscreteLinePose estimate_discrete_line_pose(float l_norm, float r_norm);
    };

}

#endif //LINE_LOOP_H
