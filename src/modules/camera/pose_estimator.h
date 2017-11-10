#ifndef NO_TORCH  // build without Torch
#ifndef POSE_ESTIMATOR_161018
#define POSE_ESTIMATOR_161018

#include <opencv2/core/core.hpp>
#include <string>
#include <vector>

#include "../../config.h"

class PoseEstimator {
public:
    PoseEstimator();
    ~PoseEstimator();

    void estimate(const cv::Mat& src);
    void drawDebug(cv::Mat& img);
    void getJoints(std::vector<cv::Point2f>& joints);

private:
    class Impl;
    Impl* impl;
};

#endif
#endif
