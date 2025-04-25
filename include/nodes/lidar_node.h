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
        leftCenter,
        rightFront,
        rightCenter,
        leftBack,
        rightBack,
        left,
        right,
        corridorRight,
        corridorLeft,
    };

    enum intersectionType
    {
        leftTurn,
        rightTurn,
        middleX,
        blindEnd,
        straightCorridor,
        T,
        TLeft,
        TRight,
    };

    enum lineReliable
    {
        reliable,
        toofar,
        notparallel,
        unreliable,
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
            if (mode == corridorRight)
                return  setPoint - lines.corridorRight.iQ;
            if (mode == corridorLeft)
                return  -(setPoint + lines.corridorLeft.iQ);
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
            if (mode == right)
                return  lines.centerRight.iK;
            if (mode == left)
                return  lines.centerLeft.iK;
            if (mode == corridorRight)
                return  lines.corridorRight.iK;
            if (mode == corridorLeft)
                return  lines.corridorLeft.iK;
            return 0;
        }

        float from_centre() const
        {
            constexpr float setPoint = 0.2;
            if (valid_line(leftFront) == nodes::lineReliable::reliable)
                return abs(setPoint+lines.leftFront.iQ);
            if (valid_line(leftBack) == nodes::lineReliable::reliable)
                return abs(setPoint+lines.leftBack.iQ);
            if (valid_line(rightFront) == nodes::lineReliable::reliable)
                return abs(setPoint-lines.rightFront.iQ);
            if (valid_line(rightBack) == nodes::lineReliable::reliable)
                return abs(setPoint-lines.rightBack.iQ);
            return 0;
        }

        lineReliable valid_line(const mode mode) const
        {
            constexpr float maxDistance = 0.35;
            constexpr float maxK = 0.4;
            switch (mode)
            {
                case leftFront:
                    if (lines.leftFront.iK > -maxK && lines.leftFront.iK < maxK )
                    {
                        if (lines.leftFront.iQ > -maxDistance)
                            return reliable;
                        return toofar;
                    }
                    if (lines.leftFront.iQ > -maxDistance)
                        return notparallel;
                    return unreliable;

                case leftBack:
                    if (lines.leftBack.iK > -maxK && lines.leftBack.iK < maxK )
                    {
                        if (lines.leftBack.iQ > -maxDistance)
                            return reliable;
                        return toofar;
                    }
                    if (lines.leftFront.iQ > -maxDistance)
                        return notparallel;
                    return unreliable;

                case rightFront:
                    if (lines.rightFront.iK > -maxK && lines.rightFront.iK < maxK )
                    {
                        if (lines.rightFront.iQ < maxDistance)
                            return reliable;
                        return toofar;
                    }
                    if (lines.rightFront.iQ < maxDistance)
                        return notparallel;
                    return unreliable;

                case rightBack:
                    if (lines.rightBack.iK > -maxK && lines.rightBack.iK < maxK )
                    {
                        if (lines.rightBack.iQ < maxDistance)
                            return reliable;
                        return toofar;
                    }
                    if (lines.rightBack.iQ < maxDistance)
                        return notparallel;
                    return unreliable;

                case leftCenter:
                    if (lines.centerLeft.iK > -maxK && lines.centerLeft.iK < maxK )
                    {
                        if (lines.centerLeft.iQ > -maxDistance)
                            return reliable;
                        return toofar;
                    }
                    if (lines.centerLeft.iQ > -maxDistance)
                        return notparallel;
                    return unreliable;

                case rightCenter:
                    if (lines.centerRight.iK > -maxK && lines.centerRight.iK < maxK )
                    {
                        if (lines.centerRight.iQ < maxDistance)
                            return reliable;
                        return toofar;
                    }
                    if (lines.centerRight.iQ < maxDistance)
                        return notparallel;
                    return unreliable;
                default: ;
            }
            return unreliable;
        }

        intersectionType get_interseptionType() const
        {
            constexpr float frontBarrier = 0.6;
            constexpr float weAreClear = 0.7;
            constexpr float TBarrier = 0.7;
            if (results.front < frontBarrier)
            {
                if (valid_line(leftFront) == nodes::lineReliable::reliable && valid_line(rightFront) == nodes::lineReliable::reliable)
                {
                    return blindEnd;
                }
                if (valid_line(leftFront) != nodes::lineReliable::reliable && valid_line(rightFront) != nodes::lineReliable::reliable)
                    return T;
                if (valid_line(leftFront) == nodes::lineReliable::reliable)
                    return rightTurn;
                if (valid_line(rightFront) == nodes::lineReliable::reliable)
                    return leftTurn;
            }
            if (valid_line(leftFront) == nodes::lineReliable::reliable && valid_line(rightFront) == nodes::lineReliable::reliable)
            {
                return straightCorridor;
            }
            if (valid_line(leftFront) == nodes::lineReliable::reliable && valid_line(rightFront) != nodes::lineReliable::reliable)
            {
                if (results.front < TBarrier)
                    return straightCorridor;
                return middleX; //TRight
            }
            if (valid_line(leftFront) != nodes::lineReliable::reliable && valid_line(rightFront) == nodes::lineReliable::reliable)
            {
                if (results.front < TBarrier)
                    return straightCorridor;
                return middleX; //TLeft
            }
            if (valid_line(leftFront) != nodes::lineReliable::reliable && valid_line(rightFront) != nodes::lineReliable::reliable)
            {
                if (results.front > weAreClear)
                    return middleX;
            }

            return straightCorridor;
        }

        bool front_equal_back(const mode mode) const
        {
            constexpr float interval = 0.05; // 1 cm tolerance
            if (mode == leftFront || mode == leftBack)
            {
                if (lines.leftFront.iQ < (lines.leftBack.iQ + interval) && lines.leftFront.iQ > (lines.leftBack.iQ - interval))
                {
                    //std::cout << "lol" << std::endl;
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

        intersectionType get_intersection() const
        {
            return lastIntersection;
        }


    private:
        algorithms::LidarFiltr filtr;

        nodes::intersectionType lastIntersection = nodes::intersectionType::straightCorridor;
        int intersectionCouter = 0;

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
            //std::cout << "Leva predni " << "y=" << lines.leftFront.iK << "x + " << lines.leftFront.iQ << std::endl;
            //std::cout << "Prava predni " << "y=" << lines.rightFront.iK << "x + " << lines.rightFront.iQ << std::endl;
            //std::cout << "Leva zadni " << "y=" << lines.leftBack.iK << "x + " << lines.leftBack.iQ << std::endl;
            //std::cout << "Prava zadni " << "y=" << lines.rightBack.iK << "x + " << lines.rightBack.iQ << std::endl;
            //std::cout << "Predni " << "y=" << lines.front.iK << "x + " << lines.front.iQ << std::endl;
            //std::cout << "Zadni " << "y=" << lines.back.iK << "x + " << lines.back.iQ << std::endl;
            //std::cout << "Leva " << "y=" << lines.centerLeft.iK << "x + " << lines.centerLeft.iQ << std::endl;
            //std::cout << "Prava " << "y=" << lines.centerRight.iK << "x + " << lines.centerRight.iQ << std::endl;
            //std::cout << "Leva coridor " << "y=" << lines.corridorLeft.iK << "x + " << lines.corridorLeft.iQ << std::endl;
            //std::cout << "Prava coridor " << "y=" << lines.corridorRight.iK << "x + " << lines.corridorRight.iQ << std::endl;


            //if (valid_line(leftFront))
                //std::cout << "Leva valid" << std::endl;
            //if (valid_line(rightFront))
                //std::cout << "Prava valid" << std::endl;
            nodes::intersectionType tmp = get_interseptionType();

            if (tmp != lastIntersection)
            {
                if (intersectionCouter > 3)
                {
                    lastIntersection = tmp;
                    intersectionCouter = 0;
                }
                intersectionCouter++;
            }
            }


    };
}

#endif //LIDAR_NODE_H
