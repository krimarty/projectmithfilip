//
// Created by student on 7.4.25.
//

#ifndef LIDAR_ALGORITHMS_H
#define LIDAR_ALGORITHMS_H
#include <cmath>
#include <vector>
#include <numeric>

namespace algorithms {

    // Structure to store filtered average distances in key directions
    struct LidarFiltrResults {
        float front;
        float back;
        float left;
        float right;
    };

    class LidarFiltr {
    public:
        LidarFiltr() = default;

        static LidarFiltrResults apply_filter(std::vector<float> points, float angle_start, float angle_end) {

            // Create containers for values in different directions
            std::vector<float> left{};
            std::vector<float> right{};
            std::vector<float> front{};
            std::vector<float> back{};

            // TODO: Define how wide each directional sector should be (in radians)
            constexpr float angle_range = M_PI / 6;

            // Compute the angular step between each range reading
            auto angle_step = (angle_end - angle_start) / points.size();

            for (size_t i = 0; i < points.size(); ++i) {
                auto angle = angle_start + i * angle_step;

                // TODO: Skip invalid (infinite) readings
                if (points[i] > 10) {
                    continue;
                }

                // TODO: Sort the value into the correct directional bin based on angle

                if (angle > (M_PI/2-angle_range/2) && (angle < M_PI/2+angle_range/2))
                {
                  right.push_back(points[i]);
                }

                else if (angle > (-M_PI/2-angle_range/2) && (angle < -M_PI/2+angle_range/2))
                {
                    left.push_back(points[i]);
                }

                else if (angle < (- M_PI + angle_range/2) || (angle > M_PI - angle_range/2))
                {
                    front.push_back(points[i]);
                }

            }

            // TODO: Return the average of each sector (basic mean filter)
            return LidarFiltrResults{
                .front = std::accumulate(front.begin(), front.end(), 0.0f)/ static_cast<float>(front.size()),
                .back = 0.0f,
                .left = std::accumulate(left.begin(), left.end(), 0.0f)/ static_cast<float>(left.size()),
                .right = std::accumulate(right.begin(), right.end(), 0.0f)/ static_cast<float>(right.size()),
            };
        }

    };
}

#endif //LIDAR_ALGORITHMS_H
