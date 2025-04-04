//
// Created by student on 10.3.25.
//

#include <algorithms/kinematics_algorithms.h>
#include <cmath>
#define R 0.033
#define L 0.14

namespace algorithms {

    KinematicsAlgorithms::KinematicsAlgorithms()= default;
    KinematicsAlgorithms::~KinematicsAlgorithms()= default;





    Coordinates  KinematicsAlgorithms::Forward_odometry (const Encoders in)
    {
        auto delta_dl = R * ((2*M_PI*in.l)/576);
        auto delta_dr = R * ((2*M_PI*in.r)/576);

        auto  delta_d = (delta_dl + delta_dr)/2;
        auto delta_fi = (delta_dr - delta_dl)/L;

        Coordinates out{};
        out.x = delta_d * std::cos(delta_fi/2);
        out.y = delta_d * std::sin(delta_fi/2);


        return out;
    }

    Encoders KinematicsAlgorithms::Inverse_odometry (const Coordinates in)
    {
        Encoders out{};


        return out;
    }

    WheelSpeed KinematicsAlgorithms::Inverse_kinematics (const RobotSpeed in)
    {
        WheelSpeed out{};
        out.r = (2*in.v + in.w*L)/(2*R);
        out.l = (2*in.v - in.w*L)/(2*R);
        return out;
    }

    RobotSpeed KinematicsAlgorithms::Forward_kinematics (const WheelSpeed in)
    {
        RobotSpeed out{};
        out.w = (R * in.r -R * in.l)/L;
        out.v = (R * in.l/2 + R * in.r/2);
        return out;
    }

    Pose KinematicsAlgorithms::update_pose(const Pose& current_pose, const Encoders in) {

        Pose out{};
        auto delta_dl = R * ((2*M_PI*in.l)/576);
        auto delta_dr = R * ((2*M_PI*in.r)/576);

        auto  delta_d = (delta_dl + delta_dr)/2;
        auto delta_fi = (delta_dr - delta_dl)/L;

        out.x = current_pose.x + delta_d * std::cos(current_pose.theta + delta_fi/2);
        out.y = current_pose.y + delta_d * std::sin(current_pose.theta + delta_fi/2);
        out.theta = current_pose.theta + delta_fi;
        return out;
    }


}
