#ifndef POSE_HG_261028
#define POSE_HG_261028

#include <opencv2/core/core.hpp>
#include <vector>

class PoseHg {
public:
    PoseHg();
    ~PoseHg();
    void estimate(const cv::Mat& src, std::vector<float>& coords);

private:
    class Impl;
    Impl *impl;
};

#endif
