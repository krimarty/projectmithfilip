//
// Created by student on 14.4.25.
//

#ifndef PLANAR_IMU_INTEGRATOR_H
#define PLANAR_IMU_INTEGRATOR_H

#include <iostream>
#include <vector>
#include <cmath>
#include <numeric>

namespace algorithms {

    class PlanarImuIntegrator {
    public:

        PlanarImuIntegrator() : theta_(0.0f), gyro_offset_(0.0f) {}

        // TODO: Call this regularly to integrate gyro_z over time
        void update(float gyro_z, double dt)
        {
            theta_ += dt * (gyro_z-gyro_offset_);
        }

        // TODO: Calibrate the gyroscope by computing average from static samples
        void setCalibration(std::vector<float> gyro)
        {
            reset();
            gyro_offset_ = std::accumulate(gyro.begin(), gyro.end(), 0.0f) / gyro.size();
        }

        // TODO: Return the current estimated yaw
        [[nodiscard]] float getYaw() const
        {
            return theta_;
        }

        // TODO: Reset orientation and calibration
        void reset()
        {
            theta_ = 0.0f;
        }

    private:
        float theta_;       // Integrated yaw angle (radians)
        float gyro_offset_; // Estimated gyro bias
    };
}


#endif //PLANAR_IMU_INTEGRATOR_H
