#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>

namespace algorithms {

    class ArucoDetector {
    public:

        // Represents one detected marker
        struct Aruco {
            int id;
            std::vector<cv::Point2f> corners;
        };

        ArucoDetector() {
            // Initialize dictionary with 4x4 markers (50 possible IDs)
            dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        }

        ~ArucoDetector() = default;

        // Detect markers in the input image
        std::vector<Aruco> detect(cv::Mat frame) {
            std::vector<Aruco> arucos;

            std::vector<int> marker_ids;
            std::vector<std::vector<cv::Point2f>> marker_corners;

            // Detect markers using OpenCV
            cv::aruco::detectMarkers(frame, dictionary_, marker_corners, marker_ids);

            // If markers were detected
            if (!marker_ids.empty()) {
                std::cout << "Arucos found: ";
                for (size_t i = 0; i < marker_ids.size(); i++) {
                    std::cout << marker_ids[i] << " ";

                    // Create Aruco struct and add to result vector
                    aruco.id = marker_ids[i];
                    aruco.corners = marker_corners[i];
                    arucos.emplace_back(aruco);
                }
                std::cout << std::endl;
            }

            return arucos;
        }

        // Accessor for the last detected frame
        cv::Mat getLastFrame() const {
            return last_frame_;
        }

        int getId() const {
            return aruco.id;
        }

        // Accessor for the last detection results
        std::vector<Aruco> getLastDetection() const {
            return last_detection_;
        }

    private:
        cv::Ptr<cv::aruco::Dictionary> dictionary_;

        Aruco aruco;

        cv::Mat last_frame_;                        // To store the latest frame
        std::vector<Aruco> last_detection_;         // To store the latest detection results
    };

}

