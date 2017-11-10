#include "face_estimator.h"

#include <math.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <omp.h>

#include "GazeEstimation.h"
#include "LandmarkCoreIncludes.h"

#include "camera.h"

namespace {
//
// These functions are copied from original and edited.
//
cv::Point3f GetPupilPosition(cv::Mat_<double> eyeLdmks3d) {
    eyeLdmks3d = eyeLdmks3d.t();
    cv::Mat_<double> irisLdmks3d = eyeLdmks3d.rowRange(0, 8);
    cv::Point3f p(cv::mean(irisLdmks3d.col(0))[0],
                  cv::mean(irisLdmks3d.col(1))[0],
                  cv::mean(irisLdmks3d.col(2))[0]);
    return p;
}

void EstimatePupils(const LandmarkDetector::CLNF& clnf_model,
                    cv::Point3f& pupil_left, cv::Point3f& pupil_right, float fx,
                    float fy, float cx, float cy) {
    const std::vector<std::string>& names = clnf_model.hierarchical_model_names;
    int part_left = -1, part_right = -1;
    size_t n = clnf_model.hierarchical_models.size();
    for (size_t i = 0; i < n; ++i) {
        if (names[i].compare("left_eye_28") == 0) part_left = i;
        if (names[i].compare("right_eye_28") == 0) part_right = i;
    }

    cv::Mat eyeLdmks3d_left =
        clnf_model.hierarchical_models[part_left].GetShape(fx, fy, cx, cy);
    pupil_left = GetPupilPosition(eyeLdmks3d_left);

    cv::Mat eyeLdmks3d_right =
        clnf_model.hierarchical_models[part_right].GetShape(fx, fy, cx, cy);
    pupil_right = GetPupilPosition(eyeLdmks3d_right);
}

void DrawGaze(cv::Mat& img, const LandmarkDetector::CLNF& clnf_model,
              const cv::Point3f& pupil_left, const cv::Point3f& pupil_right,
              const cv::Point3f& gaze_dir_left,
              const cv::Point3f& gaze_dir_right, float fx, float fy, float cx,
              float cy) {
    vector<cv::Point3d> points_left(2);
    points_left[0] = cv::Point3d(pupil_left);
    points_left[1] = cv::Point3d(pupil_left + gaze_dir_left * 50.0);

    vector<cv::Point3d> points_right(2);
    points_right[0] = cv::Point3d(pupil_right);
    points_right[1] = cv::Point3d(pupil_right + gaze_dir_right * 50.0);

    cv::Mat_<double> proj_points;
    cv::Mat_<double> mesh_0 =
        (cv::Mat_<double>(2, 3) << points_left[0].x, points_left[0].y,
         points_left[0].z, points_left[1].x, points_left[1].y,
         points_left[1].z);
    LandmarkDetector::Project(proj_points, mesh_0, fx, fy, cx, cy);
    cv::line(
        img,
        cv::Point(proj_points.at<double>(0, 0), proj_points.at<double>(0, 1)),
        cv::Point(proj_points.at<double>(1, 0), proj_points.at<double>(1, 1)),
        cv::Scalar(110, 220, 0), 2, 8);

    cv::Mat_<double> mesh_1 =
        (cv::Mat_<double>(2, 3) << points_right[0].x, points_right[0].y,
         points_right[0].z, points_right[1].x, points_right[1].y,
         points_right[1].z);
    LandmarkDetector::Project(proj_points, mesh_1, fx, fy, cx, cy);
    cv::line(
        img,
        cv::Point(proj_points.at<double>(0, 0), proj_points.at<double>(0, 1)),
        cv::Point(proj_points.at<double>(1, 0), proj_points.at<double>(1, 1)),
        cv::Scalar(110, 220, 0), 2, 8);
}

void NonOverlapingDetections(const std::vector<LandmarkDetector::CLNF>& models,
                             std::vector<cv::Rect_<double> >& face_rects,
                             float overlap_threshold = 0.5f) {
    std::vector<cv::Rect_<double> > dst_face_rects;

    for (int j = 0; j < face_rects.size(); j++) {
        bool overlapping = false;
        for (int i = 0; i < models.size(); i++) {
            const cv::Rect_<double>& model_rect = models[i].GetBoundingBox();
            float intersection_area = (model_rect & face_rects[j]).area();
            float union_area = float(model_rect.area() + face_rects[j].area()) -
                               2.f * intersection_area;

            if (intersection_area / union_area > overlap_threshold) {
                overlapping = true;
                break;
            }
        }
        if (!overlapping) {
            dst_face_rects.push_back(face_rects[j]);
        }
    }

    face_rects = dst_face_rects;
}

void NonOverlapingModels(std::vector<char>& model_activities,
                         std::vector<LandmarkDetector::CLNF>& models,
                         float overlap_threshold = 0.5f) {
    for (int i = 0; i < models.size(); i++) {
        if (!model_activities[i]) continue;
        const cv::Rect_<double>& rect1 = models[i].GetBoundingBox();

        for (int j = 0; j < i; j++) {
            if (!model_activities[j]) continue;
            const cv::Rect_<double>& rect2 = models[j].GetBoundingBox();

            float intersection_area = (rect1 & rect2).area();
            float union_area =
                float(rect1.area() + rect2.area()) - 2.f * intersection_area;

            if (intersection_area / union_area > overlap_threshold) {
                int removal_idx = (models[i].detection_certainty <=
                                   models[j].detection_certainty) ?
                                      i :
                                      j;
                model_activities[removal_idx] = false;
                models[removal_idx].Reset();
            }
        }
    }
}
}  // namespace

namespace {
// m -> cm and fix direction
inline cv::Point3f FixPositionCoord(float x, float y, float z) {
    return cv::Point3f(x / -10.f, y / -10.f, z / 10.f);
}
inline cv::Point3f FixPositionCoord(const cv::Point3f& src) {
    return FixPositionCoord(src.x, src.y, src.z);
}
// fix direction
inline cv::Point3f FixRotationCoordinate(const cv::Point3f& src) {
    return cv::Point3f(-src.x, -src.y, src.z);
}
inline cv::Vec3d FixRotationCoordinate(const cv::Vec3d& src) {
    return cv::Vec3d(-src(0), -src(1), src(2));
}
}  // namespace

// OpenFace impl pattern
class FaceEstimator::Impl {
public:
    Impl() : DETECTION_BOUNDARY(0.2), DETECTION_INTERVAL(8) {}
    ~Impl() {}

    void init(bool low_fps = false, bool replace_gaze_dir = true);
    void estimate(const cv::Mat_<uchar> gray);
    void reset();

    void getFaceResults(std::vector<char>& estimated,
                        std::vector<cv::Point3f>& angles,
                        std::vector<cv::Point3f>& positions);
    void getGazeResults(std::vector<char>& estimated,
                        std::vector<cv::Point3f>& gaze_orgs,
                        std::vector<cv::Point3f>& gaze_dirs);

    void drawDebug(cv::Mat& img);

private:
    const double DETECTION_BOUNDARY;
    const int DETECTION_INTERVAL;
    bool replace_gaze_dir;  // flag for using face angle instead of gaze

    std::vector<char> model_activities;
    std::vector<LandmarkDetector::CLNF> clnf_models;
    std::vector<LandmarkDetector::FaceModelParameters> det_parameters;

    int frame_cnt;
    float fx, fy, cx, cy;

    // results
    std::vector<char> face_estimated, gaze_estimated;
    std::vector<cv::Point3f> angles, positions;
    std::vector<cv::Point3f> gaze_dirs, gaze_orgs;

    std::vector<cv::Vec6d> raw_poses;
    std::vector<cv::Point3f> raw_gaze_dirs_left, raw_gaze_dirs_right;
    std::vector<cv::Point3f> raw_pupils_left, raw_pupils_right;
};

void FaceEstimator::Impl::init(bool low_fps, bool replace_gaze_dir) {
    printf("* Initialize FaceEstimator (n_face_max: %d)\n", N_FACE_MAX);
    this->replace_gaze_dir = replace_gaze_dir;

    // OpenFace Parameters
    LandmarkDetector::FaceModelParameters tmp_param;
    tmp_param.model_location = OPENFACE_MODEL_PATH;
    tmp_param.track_gaze = true;
    tmp_param.face_detector_location = FACE_DETECTOR_PATH;
    tmp_param.use_face_template = true;
    // This is so that the model would not try re-initialising itself
    tmp_param.reinit_video_every = -1;
    //     tmp_param.curr_face_detector =  // Low confidence but high speed
    //         LandmarkDetector::FaceModelParameters::HAAR_DETECTOR;

    // Option: Configure window size for low fps mode
    if (low_fps) {
        // make searching window size larger
        tmp_param.window_sizes_small[0] = 11;  // additional
        tmp_param.window_sizes_small[1] = 9;
        tmp_param.window_sizes_small[2] = 7;
        tmp_param.window_sizes_small[3] = 5;
    }

    // Load OpenFace model
    LandmarkDetector::CLNF tmp_model(tmp_param.model_location);
    tmp_model.face_detector_HAAR.load(tmp_param.face_detector_location);
    tmp_model.face_detector_location = tmp_param.face_detector_location;

    // Copy parameters and models
    model_activities.reserve(N_FACE_MAX);
    det_parameters.reserve(N_FACE_MAX);
    clnf_models.reserve(N_FACE_MAX);
    for (int i = 0; i < N_FACE_MAX; i++) {
        model_activities.push_back(false);
        det_parameters.push_back(tmp_param);  // copy
        clnf_models.push_back(tmp_model);     // copy
    }

    // Initialize parameters
    frame_cnt = 0;
    fx = WEBCAMERA_INTRINSIC_PARAMS[0];
    fy = WEBCAMERA_INTRINSIC_PARAMS[1];
    cx = WEBCAMERA_INTRINSIC_PARAMS[2];
    cy = WEBCAMERA_INTRINSIC_PARAMS[3];

    face_estimated.resize(N_FACE_MAX, false);
    gaze_estimated.resize(N_FACE_MAX, false);
    angles.resize(N_FACE_MAX);
    positions.resize(N_FACE_MAX);
    gaze_dirs.resize(N_FACE_MAX);
    gaze_orgs.resize(N_FACE_MAX);

    raw_poses.resize(N_FACE_MAX);
    raw_gaze_dirs_left.resize(N_FACE_MAX);
    raw_gaze_dirs_right.resize(N_FACE_MAX);
    raw_pupils_left.resize(N_FACE_MAX);
    raw_pupils_right.resize(N_FACE_MAX);
}

void FaceEstimator::Impl::estimate(const cv::Mat_<uchar> gray) {
    // Detect faces
    std::vector<cv::Rect_<double> > face_rects;
    if (++frame_cnt > DETECTION_INTERVAL) {
        frame_cnt = 0;
        // Check if all models are active
        bool all_active = true;
        for (int i = 0; i < model_activities.size(); i++) {
            if (!model_activities[i]) {
                all_active = false;
                break;
            }
        }
        if (!all_active) {
            // Detect Dlib HOG or OpenCV Cascade
            if (det_parameters[0].curr_face_detector ==
                LandmarkDetector::FaceModelParameters::HOG_SVM_DETECTOR) {
                std::vector<double> confidences;
                LandmarkDetector::DetectFacesHOG(
                    face_rects, gray, clnf_models[0].face_detector_HOG,
                    confidences);
            } else {
                LandmarkDetector::DetectFaces(
                    face_rects, gray, clnf_models[0].face_detector_HAAR);
            }
            // Remove overlapping detections
            NonOverlapingDetections(clnf_models, face_rects);
        }
    }

    // Index for next face rectangle to track
    int face_rect_idx = 0;

    // Landmark and gaze estimation for each model
    char ret_detects[clnf_models.size()];
#pragma omp parallel for
    for (int i = 0; i < clnf_models.size(); i++) {
        // If the current model has failed more than 4 times in a row, remove it
        if (clnf_models[i].failures_in_a_row > 4) {
            model_activities[i] = false;
            clnf_models[i].Reset();
        }

        // Update model
        if (model_activities[i]) {
            // Tracking
            ret_detects[i] = LandmarkDetector::DetectLandmarksInVideo(
                gray, clnf_models[i], det_parameters[i]);
        } else {
            // Exclusive process for rectangle index
            int face_rect_idx_local;
#pragma omp critical
            {
                if (face_rect_idx >= face_rects.size()) {
                    continue;  // no face to track
                }
                face_rect_idx_local = face_rect_idx++;
            }

            // Start tracking with a new detection
            clnf_models[i].Reset();
            clnf_models[i].detection_success = false;
            ret_detects[i] = LandmarkDetector::DetectLandmarksInVideo(
                gray, cv::Mat_<float>(), face_rects[face_rect_idx_local],
                clnf_models[i], det_parameters[i]);
            model_activities[i] = true;
        }
    }

    // Remove overlapping models
    NonOverlapingModels(model_activities, clnf_models);

// Calculate positions and poses for estimated landmarks
#pragma omp parallel for
    for (int i = 0; i < clnf_models.size(); i++) {
        // Check the certainty
        if (!model_activities[i] ||
            DETECTION_BOUNDARY <= clnf_models[i].detection_certainty) {
            // Failed to detect/track a face
            face_estimated[i] = false;
            gaze_estimated[i] = false;
            continue;
        }
        // Success to track face
        face_estimated[i] = true;

        // Back-Project to 3D coordinate
        raw_poses[i] = LandmarkDetector::GetCorrectedPoseWorld(clnf_models[i],
                                                               fx, fy, cx, cy);
        const cv::Vec6f& raw_pose = raw_poses[i];

        // position (Convert raw -> correct)
        positions[i] = FixPositionCoord(raw_pose[0], raw_pose[1], raw_pose[2]);
        // Convert coordinates from camera to world
        positions[i] = cvtWebCameraCoord2World(positions[i]);

        // angle (convert euler(raw_coordinate) -> eular(correct) -> rot_mat)
        cv::Mat rot_mat;
        {
            cv::Vec3d euler_angle(raw_pose[3], raw_pose[4], raw_pose[5]);
            euler_angle = FixRotationCoordinate(euler_angle);
            cv::Matx33d rot_mat_33d =
                LandmarkDetector::Euler2RotationMatrix(euler_angle);
            cv::Mat(rot_mat_33d).convertTo(rot_mat, CV_32FC1);
        }
        // Convert coordinates from camera to world
        rot_mat = getWebCameraRotationMatrix() * rot_mat;
        // Convert angle (rot_mat -> angle(rad) -> angle(degree))
        cv::Vec3f axis_angle;
        cv::Rodrigues(rot_mat, axis_angle);
        axis_angle *= 180.f / M_PI;
        angles[i] = cv::Point3f(axis_angle);

        // Gaze estimation
        gaze_estimated[i] = ret_detects[i] && det_parameters[i].track_gaze &&
                            clnf_models[i].eye_model;
        if (gaze_estimated[i]) {
            // origin
            EstimatePupils(clnf_models[i], raw_pupils_left[i],
                           raw_pupils_right[i], fx, fy, cx, cy);
            gaze_orgs[i] = FixPositionCoord(
                (raw_pupils_left[i] + raw_pupils_right[i]) * 0.5f);
            // Convert coordinates from camera to world
            gaze_orgs[i] = cvtWebCameraCoord2World(gaze_orgs[i]);

            // direction
            if (replace_gaze_dir) {
                // Use face direction instead of gaze estimation
                cv::Mat dir = (cv::Mat_<float>(3, 1) << 0, 0, -1);
                // Rotate (rot_mat is already in world coordinate)
                dir = rot_mat * dir;
                gaze_dirs[i].x = dir.at<float>(0);
                gaze_dirs[i].y = dir.at<float>(1);
                gaze_dirs[i].z = dir.at<float>(2);
            } else {
                // OpenFace estimation
                FaceAnalysis::EstimateGaze(clnf_models[i],
                                           raw_gaze_dirs_left[i], fx, fy, cx,
                                           cy, true);
                FaceAnalysis::EstimateGaze(clnf_models[i],
                                           raw_gaze_dirs_right[i], fx, fy, cx,
                                           cy, false);
                cv::Mat dir = cv::Mat(FixRotationCoordinate(
                    (raw_gaze_dirs_left[i] + raw_gaze_dirs_right[i]) * 0.5f));
                // Convert coordinates from camera to world
                dir = getWebCameraRotationMatrix() * dir;
                gaze_dirs[i].x = dir.at<float>(0);
                gaze_dirs[i].y = dir.at<float>(1);
                gaze_dirs[i].z = dir.at<float>(2);
                // Normalize the length
                gaze_dirs[i] /= cv::norm(gaze_dirs[i]);
            }
        }
    }
}

void FaceEstimator::Impl::reset() {
    for (int i = 0; i < clnf_models.size(); i++) {
        clnf_models[i].Reset();
    }
}

void FaceEstimator::Impl::getFaceResults(std::vector<char>& estimated,
                                         std::vector<cv::Point3f>& angles,
                                         std::vector<cv::Point3f>& positions) {
    estimated = this->face_estimated;
    angles = this->angles;
    positions = this->positions;
}

void FaceEstimator::Impl::getGazeResults(std::vector<char>& estimated,
                                         std::vector<cv::Point3f>& gaze_orgs,
                                         std::vector<cv::Point3f>& gaze_dirs) {
    estimated = this->gaze_estimated;
    gaze_orgs = this->gaze_orgs;
    gaze_dirs = this->gaze_dirs;
}

void FaceEstimator::Impl::drawDebug(cv::Mat& img) {
    for (int i = 0; i < clnf_models.size(); i++) {
        if (!face_estimated[i]) continue;

        // Draw landmark
        LandmarkDetector::Draw(img, clnf_models[i]);

        // Draw box
        double col =
            std::min(std::max(clnf_models[i].detection_certainty, -1.0), 1.0);
        col = (col + 1) / (DETECTION_BOUNDARY + 1.0);
        cv::Scalar box_color((1.0 - col) * 255.0, 0, col * 255);
        const int THICKNESS = 2;
        LandmarkDetector::DrawBox(img, raw_poses[i], box_color, THICKNESS, fx,
                                  fy, cx, cy);

        if (gaze_estimated[i] && !replace_gaze_dir) {
            // Draw gaze
            DrawGaze(img, clnf_models[i], raw_pupils_left[i],
                     raw_pupils_right[i], raw_gaze_dirs_left[i],
                     raw_gaze_dirs_right[i], fx, fy, cx, cy);
        }
    }
}

// === Exposed Methods ===
FaceEstimator::FaceEstimator() { impl = new Impl; }
FaceEstimator::~FaceEstimator() { delete impl; }
void FaceEstimator::init(bool low_fps, bool replace_gaze_dir) {
    impl->init(low_fps, replace_gaze_dir);
}
void FaceEstimator::estimate(const cv::Mat_<uchar> gray) {
    impl->estimate(gray);
}
void FaceEstimator::reset() { impl->reset(); }
void FaceEstimator::getFaceResults(std::vector<char>& estimated,
                                   std::vector<cv::Point3f>& angles,
                                   std::vector<cv::Point3f>& positions) {
    impl->getFaceResults(estimated, angles, positions);
}
void FaceEstimator::getGazeResults(std::vector<char>& estimated,
                                   std::vector<cv::Point3f>& gaze_orgs,
                                   std::vector<cv::Point3f>& gaze_dirs) {
    impl->getGazeResults(estimated, gaze_orgs, gaze_dirs);
}
void FaceEstimator::drawDebug(cv::Mat& img) { impl->drawDebug(img); }
