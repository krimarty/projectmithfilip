//
// Created by student on 7.4.25.
//

#ifndef LIDAR_NODE_H
#define LIDAR_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <algorithms/lidar_algorithms.h>

namespace nodes
{

    enum mode
    {
        leftFront,
        rightFront,
        leftBack,
        rightBack,
    };

    enum intersectionType
    {
        leftTurn,
        rightTurn,
        middleX,
        blindEnd,
        straightCorridor,
    };

    class LidarNode : public rclcpp::Node {
    public:

        LidarNode();

        ~LidarNode() override = default;

        float get_result() const
        {
            return results.left-results.right;
        }

        float get_error_distance(const mode mode) const
        {
            constexpr float setPoint = 0.2;
            if (mode == leftFront)
                return  -(setPoint + lines.leftFront.iQ);
            if (mode == leftBack)
                return  -(setPoint + lines.leftBack.iQ);
            if (mode == rightFront)
                return  setPoint - lines.rightFront.iQ;
            if (mode == rightBack)
                return  setPoint - lines.rightBack.iQ;
            return 0;
        }

        float get_error_angle(const mode mode) const
        {
            if (mode == leftFront)
                return  lines.leftFront.iK;
            if (mode == leftBack)
                return  lines.leftBack.iK;
            if (mode == rightFront)
                return  lines.rightFront.iK;
            if (mode == rightBack)
                return  lines.rightBack.iK;
            return 0;
        }

        float from_centre() const
        {
            constexpr float setPoint = 0.2;
            if (valid_line(leftFront))
                return abs(setPoint+lines.leftFront.iQ);
            if (valid_line(leftBack))
                return abs(setPoint+lines.leftBack.iQ);
            if (valid_line(rightFront))
                return abs(setPoint-lines.rightFront.iQ);
            if (valid_line(rightBack))
                return abs(setPoint-lines.rightBack.iQ);
            return 0;
        }

        bool valid_line(const mode mode) const
        {
            //constexpr float maxDistance = 0.15;
            constexpr float maxK = 0.4;
            switch (mode)
            {
                case leftFront:
                    //if (lines.leftFront.iQ < maxDistance)
                    if (lines.leftFront.iK > -maxK && lines.leftFront.iK < maxK )
                        return true;
                    return false;

                case leftBack:
                    //if (lines.leftBack.iQ < maxDistance)
                    if (lines.leftBack.iK > -maxK && lines.leftBack.iK < maxK )
                        return true;
                return false;

                case rightFront:
                    //if (lines.rightFront.iQ < maxDistance)
                    if (lines.rightFront.iK > -maxK && lines.rightFront.iK < maxK )
                        return true;
                return false;

                case rightBack:
                    //if (lines.rightBack.iQ < maxDistance)
                    if (lines.rightBack.iK > -maxK && lines.rightBack.iK < maxK )
                        return true;
                return false;
            }
            return false;
        }

        intersectionType get_interseptionType() const
        {
            if (results.front < 0.50) // Prekazka pred robotem
            {
                if (valid_line(leftFront) && valid_line(rightFront))
                    return middleX;
                if (valid_line(leftFront))
                    return rightTurn;
                if (valid_line(rightFront))
                    return leftTurn;
                return straightCorridor;
            }
            if (!valid_line(leftFront) && !valid_line(rightFront))
                return middleX;
            return straightCorridor;
        }

        bool front_equal_back(const mode mode) const
        {
            constexpr float interval = 0.05; // 1 cm tolerance
            if (mode == leftFront || mode == leftBack)
            {
                if (lines.leftFront.iQ < (lines.leftBack.iQ + interval) && lines.leftFront.iQ > (lines.leftBack.iQ - interval))
                {
                    std::cout << "lol" << std::endl;
                    return true;
                }
                return false;
            }

            if (mode == rightFront || mode == rightBack)
            {
                if (lines.rightFront.iQ < (lines.rightBack.iQ + interval) && lines.rightFront.iQ > (lines.rightBack.iQ - interval))
                {
                    return true;
                }
                return false;
            }

            return false;
        }

        float from_straight()
        {
            return results.front;
        }


    private:
        algorithms::LidarFiltr filtr;

        algorithms::LidarFiltrResults results{};
        algorithms::LidarLines lines{};


        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;

        void lidar_callback(std::shared_ptr<sensor_msgs::msg::LaserScan> msg)
        {
            results = algorithms::LidarFiltr::apply_filter(msg->ranges, msg->angle_min, msg->angle_max);
            if (std::isnan(results.left))
                results.left = 0;
            if (std::isnan(results.right))
                results.right = 0;

            lines = filtr.line_aprox(msg->ranges, msg->angle_min, msg->angle_max);
            std::cout << "Leva predni " << "y=" << lines.leftFront.iK << "x + " << lines.leftFront.iQ << std::endl;
            std::cout << "Prava predni " << "y=" << lines.rightFront.iK << "x + " << lines.rightFront.iQ << std::endl;
            //std::cout << "Leva zadni " << "y=" << lines.leftBack.iK << "x + " << lines.leftBack.iQ << std::endl;
            //std::cout << "Prava zadni " << "y=" << lines.rightBack.iK << "x + " << lines.rightBack.iQ << std::endl;
            }


    };
}

#endif //LIDAR_NODE_H
