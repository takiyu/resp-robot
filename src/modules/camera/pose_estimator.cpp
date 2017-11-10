#ifndef NO_TORCH  // build without Torch
#include "pose_estimator.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/opencv.h>

#include <pose-hg.h>

namespace {

// Convert dlib rectangle to OpenCV Rect
cv::Rect ConvertRectangle(const dlib::rectangle& src) {
    return cv::Rect(src.left(), src.top(), src.width(), src.height());
}

void DetectFacesDlib(const cv::Mat& src, std::vector<cv::Rect>& rects,
                     std::vector<float>& confidences,
                     dlib::frontal_face_detector& detector,
                     float resize_scaling = 1.f) {
    // Create upsampled image
    cv::Mat img = src;
    if (img.type() != CV_8U) img.convertTo(img, CV_8U);
    if (img.channels() == 3) cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
    if (resize_scaling != 1.f) {
        cv::resize(img, img, cv::Size(img.cols * resize_scaling,
                                      img.rows * resize_scaling));
    }

    // Wrap for dlib
    dlib::cv_image<uchar> dlib_img(img);

    // Detect
    std::vector<dlib::full_detection> face_detections;
    detector(dlib_img, face_detections, -0.2);

    int n_faces = face_detections.size();
    rects.resize(n_faces);
    confidences.resize(n_faces);
    for (int i = 0; i < n_faces; i++) {
        rects[i] = ConvertRectangle(face_detections[i].rect.get_rect());
        confidences[i] = face_detections[i].detection_confidence;
    }
}

}  // namespace

// impl pattern
class PoseEstimator::Impl {
public:
    Impl() { detector = dlib::get_frontal_face_detector(); }
    ~Impl() {}
    void estimate(const cv::Mat src);
    void drawDebug(cv::Mat& img);
    void getJoints(std::vector<cv::Point2f>& joints) { joints = this->joints; }

private:
    PoseHg pose_hg;
    std::vector<cv::Point2f> joints;
    dlib::frontal_face_detector detector;
};

void PoseEstimator::Impl::estimate(const cv::Mat src) {
    CV_DbgAssert(src.channels() == 3);

    // Detect faces
    std::vector<cv::Rect> face_rects;
    std::vector<float> face_confidences;
    DetectFacesDlib(src, face_rects, face_confidences, this->detector);
    if (face_rects.size() == 0) {
        joints.resize(0);  // Failed to detect
        return;
    }

    // Create pose input image
    cv::Mat pose_img;
    if (src.type() == CV_32F) {
        pose_img = src.clone();
    } else {
        src.convertTo(pose_img, CV_32F);
    }
    cv::cvtColor(pose_img, pose_img, cv::COLOR_BGR2RGB);
    cv::normalize(pose_img, pose_img, 0.0, 1.0, CV_MINMAX);

    // TODO Support multi faces
    cv::Rect face_rect = face_rects[0];

    // Transform image using a face rectangle
    // (Input size is defined by the network)
    cv::Point2f face_tl = face_rect.tl();
    cv::Point2f src_pts[3] = {face_tl,
                              face_tl + cv::Point2f(face_rect.width, 0.f),
                              face_tl + cv::Point2f(0.f, face_rect.height)};
    const cv::Point2f DST_FACE_CENTER(256.f / 2.f, 60.f);
    const float DST_FACE_W = 40.f;
    const float DST_FACE_H = 40.f;
    const cv::Point2f DST_FACE_TL =
        DST_FACE_CENTER - cv::Point2f(DST_FACE_W / 2.f, DST_FACE_H / 2.f);
    cv::Point2f dst_pts[3] = {
        DST_FACE_TL, DST_FACE_TL + cv::Point2f(DST_FACE_W, 0.f),
        DST_FACE_TL + cv::Point2f(0.f, DST_FACE_H),
    };
    cv::Mat M = cv::getAffineTransform(src_pts, dst_pts);
    M.convertTo(M, CV_32FC1);  // double -> float
    cv::warpAffine(pose_img, pose_img, M, cv::Size(256, 256));
    //     // [DEBUG] show pose image
    //     cv::Mat pose_img_show;
    //     cv::cvtColor(pose_img, pose_img_show, cv::COLOR_RGB2BGR);
    //     cv::imshow("pose_img", pose_img_show);

    // Forward network
    std::vector<float> local_coords;
    pose_hg.estimate(pose_img, local_coords);

    // Prepare for de-transformation
    int n_joint = local_coords.size() / 2;
    cv::Mat local_coords_mat(local_coords);                   //    [32 x 1]
    local_coords_mat = local_coords_mat.reshape(1, n_joint);  // -> [16 x 2]
    local_coords_mat = local_coords_mat.t();                  // -> [2 x 16]
    // Convert to homogeneous coordinates
    cv::Mat ones = cv::Mat::ones(1, n_joint, CV_32FC1);
    local_coords_mat.push_back(ones);  // -> [3 x 16]

    // De-transform
    cv::Mat m_3 = (cv::Mat_<float>(1, 3) << 0, 0, 1);
    M.push_back(m_3);
    cv::Mat detransed_coords = M.inv() * local_coords_mat;  //    [3 x 16]

    // Convert to
    this->joints.resize(n_joint);
    for (int i = 0; i < n_joint; i++) {
        float& sx = local_coords_mat.at<float>(0, i);
        float& sy = local_coords_mat.at<float>(1, i);
        if (sx >= 0 && sy >= 0) {
            this->joints[i].x = detransed_coords.at<float>(0, i);
            this->joints[i].y = detransed_coords.at<float>(1, i);
        } else {
            this->joints[i].x = -1.f;
            this->joints[i].y = -1.f;
        }
    }
}

void PoseEstimator::Impl::drawDebug(cv::Mat& img) {
    // Check current pose
    if (joints.size() != 16) return;

    // Draw lines
    int joint_idxs[14][2] = {{0, 1},   {1, 2},  {2, 6},   {3, 4},  {3, 6},
                             {4, 5},   {6, 8},  {8, 9},   {13, 8}, {10, 11},
                             {11, 12}, {12, 8}, {13, 14}, {14, 15}};
    for (int i = 0; i < 14; i++) {
        cv::Point2f& j0 = this->joints[joint_idxs[i][0]];
        cv::Point2f& j1 = this->joints[joint_idxs[i][1]];
        if (j0.x < 0 || j0.y < 0 || j1.x < 0 || j1.y < 0) continue;
        cv::line(img, j0, j1, cv::Scalar(0, 255, 0), 2);
    }

    // Draw joint points
    for (int i = 0; i < this->joints.size(); i++) {
        cv::Point2f& j0 = this->joints[i];
        if (j0.x < 0 || j0.y < 0) continue;
        cv::circle(img, j0, 4, cv::Scalar(255, 0, 0), -1);
    }
}

// ===== exposed methods =====
PoseEstimator::PoseEstimator() { impl = new Impl; }
PoseEstimator::~PoseEstimator() { delete impl; }
void PoseEstimator::estimate(const cv::Mat& src) { impl->estimate(src); }
void PoseEstimator::drawDebug(cv::Mat& img) { impl->drawDebug(img); }
void PoseEstimator::getJoints(std::vector<cv::Point2f>& joints) {
    impl->getJoints(joints);
}

#endif
