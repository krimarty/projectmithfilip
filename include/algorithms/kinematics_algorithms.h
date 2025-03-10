//
// Created by student on 10.3.25.
//

#ifndef ESTIMATION_ALGORITHM_H
#define ESTIMATION_ALGORITHM_H
#include <utility>

namespace algorithms {
    class KinematicsAlgorithms{
    public:
        // Constructor
        KinematicsAlgorithms();
        // Destructor (default)
        ~KinematicsAlgorithms();

        void forward_kinematics();
        void inverse_kinematics();


        void estimate();
        void inverse_estimate();

        std::pair<float, float> Forward_odometry (std::pair<int, int> p);
        std::pair<float, float> Inverse (std::pair<float, float> p);
        std::pair<float, float> Inverse_kinematics (std::pair<float, float> p);
        std::pair<float, float> Forward_kinematics (std::pair<float, float> p);

    private:
        // Variable to store the last received button press value
        int iX;
        int iY;
        // Subscriber for button press messages

    };
}

#endif //ESTIMATION_ALGORITHM_H
