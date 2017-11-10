#ifndef CAMERA_160705
#define CAMERA_160705

#include <boost/thread.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../../config.h"

cv::Mat getWebCameraIntrinsicMatrix();
cv::Mat getWebCameraRotationMatrix();
cv::Point3f cvtWebCameraCoord2World(const cv::Point3f& pos);
void setWebCameraDynamicAngle(const cv::Point3f& angle);  // for dynamic

class WebCamera {
public:
    WebCamera() : capture_thread(NULL) {}
    ~WebCamera() { close(); }

    bool open(bool use_capture_thread = WEBCAMERA_USE_CAPTURE_THREAD);
    void close();
    cv::Mat capture();

private:
    cv::VideoCapture cap;
    boost::thread* capture_thread;
    cv::Mat captured_img;

    cv::VideoWriter writer;
};

#endif
