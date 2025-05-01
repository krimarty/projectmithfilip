//
// Created by student on 7.4.25.
//

#ifndef LIDAR_ALGORITHMS_H
#define LIDAR_ALGORITHMS_H
#include <cmath>
#include <vector>
#include <numeric>

namespace algorithms {

    struct line
    {
        float iK; //smernice
        float iQ; //offset
    };

    // Structure to store filtered average distances in key directions
    struct LidarFiltrResults {
        float front;
        float back;
        float left;
        float right;
    };

    struct LidarLines {
        line leftFront;
        line rightFront;
        line centerLeft;
        line leftBack;
        line rightBack;
        line centerRight;
        line front;
        line back;
        line corridorLeft;
        line corridorRight;
    };

    struct Point2D {
        float x;
        float y;
    };

    class LidarFiltr {
    public:
        LidarFiltr() = default;

        line fitLineLeastSquares(const std::vector<Point2D>& points) {
            float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
            int n = points.size();

            if (n < 2) return {0, 0}; // nelze fitovat

            for (const auto& pt : points) {
                sumX += pt.x;
                sumY += pt.y;
                sumXY += pt.x * pt.y;
                sumX2 += pt.x * pt.x;
            }

            float denominator = n * sumX2 - sumX * sumX;
            if (denominator == 0) return {0, 0}; // vertikální přímka?

            float k = (n * sumXY - sumX * sumY) / denominator;
            float q = (sumY - k * sumX) / n;

            return {k, q};
        }

        LidarLines line_aprox(std::vector<float> points, float angle_start, float angle_end)
        {
            std::vector<Point2D> leftFront, rightFront, leftBack, rightBack, centerLeft, centerRight, front, back, corridorLeft, corridorRight;

            auto angle_step = (angle_end - angle_start) / points.size();

            //kouka bliz
            constexpr float bigAnglecorridor = 1.107148718;
            constexpr float smallAnglecorridor = 0.6747409422;
            //kouka dal
            constexpr float bigAngle = 0.7853981634;
            constexpr float smallAngle = 0.5191461142;
            constexpr float frontAngle = 0.358770673;

            for (size_t i = 0; i < points.size(); ++i) {
                auto angle = angle_start + i * angle_step;
                float r = points[i];

                // Skip invalid (infinite) readings
                if (points[i] > 10) {
                    continue;
                }

                float x = r * std::cos(angle);
                float y = r * std::sin(angle);

                // Dělení na sektory – jednoduché rozdělení podle kvadrantů
                if (angle > M_PI-bigAngle && angle < M_PI-smallAngle) rightFront.push_back({x, y});
                if (angle > (-M_PI+smallAngle) && angle < (-M_PI+bigAngle) ) leftFront.push_back({x, y});
                if (angle < 0-smallAngle && angle > 0-bigAngle) leftBack.push_back({x, y});
                if (angle > 0+smallAngle && angle < 0+bigAngle) rightBack.push_back({x, y});
                if (angle > M_PI-frontAngle || angle < -M_PI+frontAngle) front.push_back({x, y});
                if (angle < frontAngle && angle > -frontAngle) back.push_back({x, y});
                if (angle > -(M_PI/2)-frontAngle && angle < -(M_PI/2)+frontAngle) centerLeft.push_back({x, y});
                if (angle < (M_PI/2)+frontAngle && angle > (M_PI/2)-frontAngle) centerRight.push_back({x, y});

                if (angle > M_PI-bigAnglecorridor && angle < M_PI-smallAnglecorridor) corridorRight.push_back({x, y});
                if (angle > (-M_PI+smallAnglecorridor) && angle < (-M_PI+bigAnglecorridor) ) corridorLeft.push_back({x, y});


            }

            return {
                .leftFront = fitLineLeastSquares(leftFront),
                .rightFront = fitLineLeastSquares(rightFront),
                .centerLeft = fitLineLeastSquares(centerLeft),
                .leftBack = fitLineLeastSquares(leftBack),
                .rightBack = fitLineLeastSquares(rightBack),
                .centerRight = fitLineLeastSquares(centerRight),
                .front = fitLineLeastSquares(front),
                .back = fitLineLeastSquares(back),
                .corridorLeft = fitLineLeastSquares(corridorLeft),
                .corridorRight = fitLineLeastSquares(corridorRight)

            };
        }

        static LidarFiltrResults apply_filter(std::vector<float> points, float angle_start, float angle_end) {

            // Create containers for values in different directions
            std::vector<float> left{};
            std::vector<float> right{};
            std::vector<float> front{};
            std::vector<float> back{};

            // TODO: Define how wide each directional sector should be (in radians)
            constexpr float angle_range = M_PI / 20;

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
