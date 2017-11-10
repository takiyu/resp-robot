#include "config.h"

#include <glm/gtx/transform.hpp>
#include <opencv2/highgui/highgui.hpp>

/* ================================= Webcam ================================= */
const int WEBCAMERA_ID = 1;
// const int WEBCAMERA_ID = 0;
const int WEBCAMERA_SIZE[2] = {848, 480};  // 16:9
// const int WEBCAMERA_SIZE[2] = {640, 480};   // 4:3
const int WEBCAMERA_FPS = 30;
const float WEBCAMERA_INTRINSIC_PARAMS[4] = {  // [fx, fy, cx, cy]
    // Dell XPS Webcam [640 x 480]
    // 7.0279330882735348e+02, 7.0279330882735348e+02, 3.1950000000000000e+02,
    // 2.3950000000000000e+02};
    // // Dell XPS Webcam [1280 x 720]
    // 9.6725227395451304e+02, 9.6725227395451304e+02, 6.3950000000000000e+02,
    // 3.5950000000000000e+02};
    // PS3Eye [640 x 480]
    // 5.3650404066615113e+02, 5.3650404066615113e+02, 3.1950000000000000e+02,
    // 2.3950000000000000e+02};
    // Logicool c920r [848 x 480]
    5.6032160242049554e+02, 3.9950000000000000e+02, 5.6032160242049554e+02,
    2.2350000000000000e+02};
// const bool WEBCAMERA_DYNAMIC_ON_ROBOVIE = false;
const bool WEBCAMERA_DYNAMIC_ON_ROBOVIE = true;
// const cv::Point3f WEBCAMERA_POSITION(0.f, 54.f, -8.f);  // outer
const cv::Point3f WEBCAMERA_POSITION(0.f, 14.f, 5.f);   // on robovie
// const cv::Point3f WEBCAMERA_ANGLES(-23.f, -14.f, 0.f);  // outer
const cv::Point3f WEBCAMERA_ANGLES(5.f, -14.f, 0.f);    // on robovie
const bool WEBCAMERA_USE_CAPTURE_THREAD = true;    // needs for output
const std::string WEBCAMERA_OUTPUT_FILENAME = "";  // empty is no output
const int WEBCAMERA_OUTPUT_CODEC = cv::VideoWriter::fourcc('F', 'M', 'P', '4');

/* ============================= Face Estimator ============================= */
const std::string OPENFACE_MODEL_PATH =
    "../3rdParty/OpenFace/lib/LandmarkDetector/model/main_clnf_general.txt";
const std::string FACE_DETECTOR_PATH =
    //     "/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml";
    "/usr/local/share/OpenCV/haarcascades/haarcascade_frontalface_alt.xml";
const int N_FACE_MAX = 6;
const bool FACE_LOW_FPS = false;
const bool FACE_REPLACE_GAZE_DIR = true;

/* ============================ Object Detector ============================= */
const std::string OBJECT_DETECT_HOG_PATHS[N_OBJECT_DETECT] =
    // The order must be the same as training. Their indices are important.
    {"../data/object_hog/apple.hog", "../data/object_hog/green_apple.hog",
     "../data/object_hog/hand.hog"};
const std::string OBJECT_CHIP_CLASSIFY_SVM_PATH =
    "../data/object_hog/classify.svm";
const float OBJECT_3D_SIZES[N_OBJECT_DETECT][2] = {
    {7.85f, 8.25f}, {7.90f, 7.10f}, {13.00f, 14.00f},
};  // {width, height}
const cv::Scalar OBJECT_VIEW_COLORS[N_OBJECT_DETECT] = {
    cv::Scalar(0, 0, 255), cv::Scalar(0, 255, 0), cv::Scalar(150, 150, 255)};

/* ============================== Camera Scene ============================== */
const std::string MESH_FACE_PATH = "../3rdParty/mesh/face.obj";
const std::string MESH_ROBOVIE_PATH = "../3rdParty/mesh/humanoid_head.obj";
const std::string MESH_OBJECT_PATHS[N_OBJECT_DETECT] = {
    "../3rdParty/mesh/apple.obj", "../3rdParty/mesh/apple.obj",
    "../3rdParty/mesh/Hand.obj",
};
const float VIEW_ATTENSION_COLOR_SCALE = 2.f;
const cv::Scalar ROBOVIE_VIEW_COLOR(255, 255, 255);
const float ATTENTION_MAX_DEPTH = 200.f;
const float ATTENTION_MAX_DEGREE = 15.f;
const float ATTENTION_ERROR_COEFF_THETA = 1.f;
const float ATTENTION_ERROR_COEFF_DEPTH = 1.f;
const float ATTENTION_BACK_DEPTH = 300.f;

/* ================================== Audio ================================= */
const int PA_FRAMES_PER_BUFFER = 16;
const std::string PA_INPUT_DEV_KEY = "default";
const std::string PA_OUTPUT_DEV_KEY = "default";

/* ================================== VAD ================================== */
const float VAD_MIC_SCALE_DEFALUT = 1.0;
const float VAD_MIN_NONACTIVE_MS = 200.f;

/* =========================== Voice Recognition ============================ */
const int N_JULIUS_CONFIG = 2;
const std::string JULIUS_CONFIGS[] = {
    "../3rdParty/julius_models/main.jconf",
    "../3rdParty/julius_models/am-dnn.jconf",
};

/* ============================ Voice Synthesis ============================= */
const std::string OPEN_JTALK_DICT =
    "../3rdParty/open_jtalk_models/open_jtalk_dic_utf_8-1.09";
const std::string OPEN_JTALK_HTS_VOICE =
    "../3rdParty/open_jtalk_models/mei/mei_normal.htsvoice";

/* ============================= Robovie Serial ============================= */
const std::string ROBOVIE_DEV_NAME = "/dev/ttyUSB0";
const std::map<std::string, int> ROBOVIE_MOTOR_IDXS = {
    // {label, index}   (index must be [0:ROBOVIE_N_MOTOR] and sequential)
    {"R arm 0", 0},   {"R arm 1", 1},   {"R arm 2", 2},  {"R arm 3", 3},
    {"L arm 0", 4},   {"L arm 1", 5},   {"L arm 2", 6},  {"L arm 3", 7},
    {"Head z", 8},    {"Head x", 9},    {"Head y", 10},  {"Body", 11},
    {"L eye y", 12},  {"L eyelid", 13}, {"L eye x", 14}, {"R eye y", 15},
    {"R eyelid", 16}, {"R eye x", 17},
};
const float ROBOVIE_MOTOR_RANGES[ROBOVIE_N_MOTOR][2] = {
    // {raw_min, raw_max}
    {-87.f, 87.f},  // right arm 0
    {-87.f, 85.f},  // right arm 1
    {-86.f, 87.f},  // right arm 2
    {-86.f, 86.f},  // right arm 3
    {-87.f, 87.f},  // left arm 0
    {-85.f, 87.f},  // left arm 1
    {-86.f, 87.f},  // left arm 2
    {-86.f, 87.f},  // left arm 3
    {-27.f, 20.f},  // head z (left/right)
    {-27.f, 27.f},  // head x (front/back)
    {-86.f, 86.f},  // head y (twist)
    {-87.f, 87.f},  // body
    {-19.f, 44.f},  // left eye y (left/right)
    {-70.f, 58.f},  // left eyelid
    {-36.f, 28.f},  // left eye x (up/down)
    {-33.f, 12.f},  // right eye y (left/right)
    {-60.f, 69.f},  // right eyelid
    {-36.f, 47.f},  // right eye x (up/down)
};
const float ROBOVIE_MOTOR_NORM_PARAMS[ROBOVIE_N_MOTOR][2] = {
    // {scale, offset} :  deg_value * scale + offset = raw_value
    {-76.0f / 90.f, 6.f},   // right arm 0
    {-143.f / 90.f, -26.f},   // right arm 1
    {-56.0f / 45.f, 15.f},     // right arm 2
    {117.0f / 90.f, 6.f},    // right arm 3
    {79.00f / 90.f, -3.f},    // left arm 0
    {-147.f / 90.f, 7.f},    // left arm 1
    {-54.0f / 45.f, -20.f},    // left arm 2
    {-118.f / 90.f, -6.f},   // left arm 3
    {23.00f / 30.f, -3.f},    // head z (left/right)
    {-54.0f / 45.f, -18.f},     // head x (front/back)
    {79.00f / 90.f, 0.f},     // head y (twist)
    {75.00f / 90.f, 0.f},     // body
    {44.00f / 45.f, -20.f},     // left eye y (left/right)
    {-126.f / 90.f, -128.f},  // left eyelid
    {-36.0f / 30.f, -4.f},     // left eye x (up/down)
    {33.00f / 15.f, 12.f},     // right eye y (left/right)
    {119.0f / 90.f, 125.f},   // right eyelid
    {43.00f / 30.f, 20.f},     // right eye x (up/down)
};
const float ROBOVIE_MOTOR_DEFAULT_MAX_SPEED = 4.f;
const float ROBOVIE_MOTOR_DEFAULT_ACCEL = 0.4f;

/* ============================= Robovie Action ============================= */
// 3D coordinates
// Direction variables are not configurable now because of Euler angles.
// To configure robovie angles, please edit ROBOVIE_TRANSFORM_ORG_TO_BODY.
const glm::vec3 ROBOVIE_ORG_POS(0, 0, 0);
const glm::vec3 ROBOVIE_ORG_DIR(glm::normalize(glm::vec3(0, 0, 1)));
const glm::vec3 ROBOVIE_ORG_DIR_ARM(glm::normalize(glm::vec3(0, -1, 0)));
// 3D homogeneous-coordinates
const glm::vec4 ROBOVIE_ORG_POS_HM = glm::vec4(ROBOVIE_ORG_POS, 1);
const glm::vec4 ROBOVIE_ORG_DIR_HM = glm::vec4(ROBOVIE_ORG_DIR, 0);
const glm::vec4 ROBOVIE_ORG_DIR_ARM_HM = glm::vec4(ROBOVIE_ORG_DIR_ARM, 0);
// Parts transformation matrix
const glm::mat4 ROBOVIE_TRANSFORM_ORG_TO_BODY =
    glm::translate(glm::vec3(0, 0, 0)) *  // The origin can be configure here.
    glm::rotate(glm::radians(0.f), glm::vec3(0, 1, 0));
const glm::mat4 ROBOVIE_TRANSFORM_BODY_TO_HEAD =
    glm::translate(glm::vec3(0, 30, 0));
const glm::mat4 ROBOVIE_TRANSFORM_HEAD_TO_LEFT_EYE =
    glm::translate(glm::vec3(2.8, 8, 0));
const glm::mat4 ROBOVIE_TRANSFORM_HEAD_TO_RIGHT_EYE =
    glm::translate(glm::vec3(-2.8, 8, 0));
const glm::mat4 ROBOVIE_TRANSFORM_BODY_TO_LEFT_ARM =
    glm::translate(glm::vec3(9, 27, 2));
const glm::mat4 ROBOVIE_TRANSFORM_LEFT_ARM_01_TO_23 =
    glm::translate(glm::vec3(0, -9, 0));
const glm::mat4 ROBOVIE_TRANSFORM_BODY_TO_RIGHT_ARM =
    glm::translate(glm::vec3(-9, 27, 2));
const glm::mat4 ROBOVIE_TRANSFORM_RIGHT_ARM_01_TO_23 =
    glm::translate(glm::vec3(0, -9, 0));

/* ================================ Plotter ================================= */
const double PLOTTER_PNG_WAIT_MS = 10.0;
