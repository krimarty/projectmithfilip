//
// Created by student on 17.3.25.
//

#ifndef LINE_NODE_H
#define LINE_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>

#define L_MIN 50
#define L_MAX 990
#define R_MIN 50
#define R_MAX 1010

#define LSM_A (-0.1507)
#define LSM_B 0


namespace nodes {
    enum class DiscreteLinePose {
        LineOnLeft,
        LineOnRight,
        LineNone,
        LineBoth,
    };

    class LineNode : public rclcpp::Node {
    public:

        LineNode();

        ~LineNode() override = default;

        // relative pose to line in meters
        float get_continuous_line_pose()
        {
            return estimate_continuous_line_pose(l_sensor, r_sensor);
        }

        DiscreteLinePose get_discrete_line_pose()
        {
            return estimate_discrete_line_pose(l_sensor, r_sensor);
        }


    private:

        float l_sensor;
        float r_sensor;

        rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr line_sensors_subscriber_;

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



#endif //LINE_NODE_H
