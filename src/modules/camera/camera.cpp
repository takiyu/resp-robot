#include "camera.h"

#include <opencv2/calib3d/calib3d.hpp>

namespace {
// Convert angle to rotation matrix
inline cv::Mat cvtAngle2RotMat(const cv::Point3f& angle) {
    cv::Mat angle_mat = (cv::Mat_<float>(1, 3) << angle.x, angle.y, angle.z);
    angle_mat *= M_PI / 180.f;
    cv::Rodrigues(angle_mat, angle_mat);
    CV_DbgAssert(angle_mat.type() == CV_32FC1);
    return angle_mat;
}

// Convert position to translation matrix
inline cv::Mat cvtPosition2TransMat(const cv::Point3f& pos) {
    return (cv::Mat_<float>(3, 1) << pos.x, pos.y, pos.z);
}

inline cv::Mat initIntrinsicMatrix() {
    return (cv::Mat_<float>(3, 3) << WEBCAMERA_INTRINSIC_PARAMS[0], 0,
            WEBCAMERA_INTRINSIC_PARAMS[2], 0, WEBCAMERA_INTRINSIC_PARAMS[1],
            WEBCAMERA_INTRINSIC_PARAMS[3], 0, 0, 1);
}

// TODO: Move to WebCamera class
// Webcamera parameters
static cv::Mat intrinsic_mat = initIntrinsicMatrix();
static cv::Mat extrinsic_rot_mat = cvtAngle2RotMat(WEBCAMERA_ANGLES);
static cv::Mat extrinsic_trans_mat = cvtPosition2TransMat(WEBCAMERA_POSITION);
static cv::Mat dynamic_rot_mat = cv::Mat::eye(3, 3, CV_32FC1);

}  // namespace

cv::Mat getWebCameraIntrinsicMatrix() { return intrinsic_mat; }

cv::Mat getWebCameraRotationMatrix() {
    // Switch static or dynamic
    if (WEBCAMERA_DYNAMIC_ON_ROBOVIE) {
        return dynamic_rot_mat * extrinsic_rot_mat;  // Dynamic
    } else {
        return extrinsic_rot_mat;  // Static
    }
}

cv::Point3f cvtWebCameraCoord2World(const cv::Point3f& pos) {
    // Switch static or dynamic
    cv::Mat res = extrinsic_rot_mat * cv::Mat(pos) + extrinsic_trans_mat;
    if (WEBCAMERA_DYNAMIC_ON_ROBOVIE) {
        return cv::Point3f(cv::Mat(dynamic_rot_mat * res));  // Dynamic
    } else {
        return cv::Point3f(res);  // Static
    }
}

void setWebCameraDynamicAngle(const cv::Point3f& angle) {
    dynamic_rot_mat = cvtAngle2RotMat(angle);
}

bool WebCamera::open(bool use_capture_thread) {
    printf("* Open camera (id: %d)\n", WEBCAMERA_ID);
    cap.open(WEBCAMERA_ID);
    if (!cap.isOpened()) {
        printf(" >> failed\n");
        return false;
    }

    // Set parameters
    cap.set(CV_CAP_PROP_FRAME_WIDTH, WEBCAMERA_SIZE[0]);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, WEBCAMERA_SIZE[1]);
    cap.set(CV_CAP_PROP_FPS, WEBCAMERA_FPS);

    // Start camera capture thread
    if (use_capture_thread) {
        capture_thread = new boost::thread([&]() {
            printf("* Start a camera capture thread\n");
            float sec = 1.f / float(WEBCAMERA_FPS);
            boost::posix_time::milliseconds ms_boost(sec * 1000.f);
            while (true) {
                boost::this_thread::sleep(ms_boost);

                // Capture
                // cap.grab();
                cap >> captured_img;

                // Output
                if (!WEBCAMERA_OUTPUT_FILENAME.empty()) {
                    bool ret = writer.isOpened();
                    if (!ret) {
                        printf("* Open video writer: %s\n",
                               WEBCAMERA_OUTPUT_FILENAME.c_str());
                        ret = writer.open(WEBCAMERA_OUTPUT_FILENAME,
                                          WEBCAMERA_OUTPUT_CODEC, WEBCAMERA_FPS,
                                          captured_img.size());
                    }
                    if (ret) {
                        writer << captured_img;
                    } else {
                        printf(" >> Failed to open video writer\n");
                    }
                }
            }
        });
    }
    return true;
}

void WebCamera::close() {
    if (capture_thread != NULL) {
        printf("* Exit a camera capturing thread\n");
        capture_thread->interrupt();
        capture_thread->join();
        delete capture_thread;
        capture_thread = NULL;
    }
    if (cap.isOpened()) {
        printf("* Close camera\n");
        cap.release();
    }
}

cv::Mat WebCamera::capture() {
    if (captured_img.empty()) {
        return cv::Mat::zeros(100, 100, CV_8UC3) + cv::Scalar(0, 255, 0);
    } else {
        return captured_img;
    }
}
