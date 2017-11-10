#ifndef FACE_ESTIMATOR_160628
#define FACE_ESTIMATOR_160628

#include <opencv2/core/core.hpp>
#include <string>
#include <vector>

#include "../../config.h"

class FaceEstimator {
public:
    FaceEstimator();
    ~FaceEstimator();

    void init(bool low_fps = FACE_LOW_FPS,
              bool replace_gaze_dir = FACE_REPLACE_GAZE_DIR);
    void estimate(const cv::Mat_<uchar> gray);
    void reset();

    void getFaceResults(std::vector<char> &estimated,
                        std::vector<cv::Point3f> &angles,
                        std::vector<cv::Point3f> &positions);
    void getGazeResults(std::vector<char> &estimated,
                        std::vector<cv::Point3f> &gaze_orgs,
                        std::vector<cv::Point3f> &gaze_dirs);

    void drawDebug(cv::Mat &img);

private:
    class Impl;
    Impl *impl;
};

#endif
