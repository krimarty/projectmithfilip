//
// Created by student on 10.3.25.
//

#include <algorithms/kinematics_algorithms.h>
#include <cmath>
#define R 0.033
#define L 0.14

namespace algorithms {

    KinematicsAlgorithms::KinematicsAlgorithms(){}
    KinematicsAlgorithms::~KinematicsAlgorithms(){}



    std::pair<float, float> KinematicsAlgorithms::Forward_odometry (std::pair<int, int> p)
    {
        float delta_dl = R * (2*M_PI/576*p.first );
        float delta_dr = R * (2*M_PI/576*p.second );

        float  delta_d = (delta_dl + delta_dr)/2;
        float delta_fi = (delta_dl - delta_dr)/L;

        float delta_x = delta_d * std::cos(delta_fi/2);
        float delta_y = delta_d * std::sin(delta_fi/2);

        return std::make_pair(delta_x, delta_y);
    }

    std::pair<float, float> KinematicsAlgorithms::Inverse_kinematics (std::pair<float, float> p)
    {
      float v_r = (2*p.first + p.second*L)/(2*R);
      float v_l = (2*p.first - p.second*L)/(2*R);
      return std::make_pair(v_l, v_r);
    }


    std::pair<float, float> KinematicsAlgorithms::Inverse (std::pair<float, float> p)
    {


      return std::make_pair(p.second, p.first);
    }

    std::pair<float, float> KinematicsAlgorithms::Forward_kinematics (std::pair<float, float> p)
    {
        float w = (R * p.first -R * p.second)/L;
        float v = (R * p.first/2 + R * p.second/2);
        return std::make_pair(v, w);
    }

}
