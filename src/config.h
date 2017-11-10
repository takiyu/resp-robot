#ifndef CONFIG_H_161005
#define CONFIG_H_161005

#include <map>
#include <string>

#include <opencv2/core/core.hpp>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>

/* ================================= Webcam ================================= */
extern const int WEBCAMERA_ID;
extern const int WEBCAMERA_SIZE[2];
extern const int WEBCAMERA_FPS;
extern const float WEBCAMERA_INTRINSIC_PARAMS[4];
extern const bool WEBCAMERA_DYNAMIC_ON_ROBOVIE;
extern const cv::Point3f WEBCAMERA_POSITION;
extern const cv::Point3f WEBCAMERA_ANGLES;
extern const bool WEBCAMERA_USE_CAPTURE_THREAD;
extern const std::string WEBCAMERA_OUTPUT_FILENAME;
extern const int WEBCAMERA_OUTPUT_CODEC;

/* ============================= Face Estimator ============================= */
extern const std::string OPENFACE_MODEL_PATH;
extern const std::string FACE_DETECTOR_PATH;
extern const int N_FACE_MAX;
extern const bool FACE_LOW_FPS;
extern const bool FACE_REPLACE_GAZE_DIR;

/* ============================ Object Detector ============================= */
const int N_OBJECT_DETECT = 3;
extern const std::string OBJECT_DETECT_HOG_PATHS[N_OBJECT_DETECT];
extern const std::string OBJECT_CHIP_CLASSIFY_SVM_PATH;
extern const float OBJECT_3D_SIZES[N_OBJECT_DETECT][2];
extern const cv::Scalar OBJECT_VIEW_COLORS[N_OBJECT_DETECT];

/* ============================== Camera Scene ============================== */
extern const std::string MESH_FACE_PATH;
extern const std::string MESH_ROBOVIE_PATH;
extern const std::string MESH_OBJECT_PATHS[N_OBJECT_DETECT];
extern const float VIEW_ATTENSION_COLOR_SCALE;
extern const cv::Scalar ROBOVIE_VIEW_COLOR;
extern const float ATTENTION_MAX_DEPTH;
extern const float ATTENTION_MAX_DEGREE;
extern const float ATTENTION_ERROR_COEFF_THETA;
extern const float ATTENTION_ERROR_COEFF_DEPTH;
extern const float ATTENTION_BACK_DEPTH;

/* ================================== Audio ================================= */
extern const int PA_FRAMES_PER_BUFFER;
extern const std::string PA_INPUT_DEV_KEY;
extern const std::string PA_OUTPUT_DEV_KEY;

/* ================================== VAD ================================== */
extern const float VAD_MIC_SCALE_DEFALUT;
extern const float VAD_MIN_NONACTIVE_MS;

/* =========================== Voice Recognition ============================ */
extern const int N_JULIUS_CONFIG;
extern const std::string JULIUS_CONFIGS[];

/* ============================ Voice Synthesis ============================= */
extern const std::string OPEN_JTALK_DICT;
extern const std::string OPEN_JTALK_HTS_VOICE;

/* ============================= Robovie Serial ============================= */
const int ROBOVIE_N_MOTOR = 18;
extern const std::string ROBOVIE_DEV_NAME;
extern const std::map<std::string, int> ROBOVIE_MOTOR_IDXS;
extern const float ROBOVIE_MOTOR_RANGES[ROBOVIE_N_MOTOR][2];
extern const float ROBOVIE_MOTOR_NORM_PARAMS[ROBOVIE_N_MOTOR][2];
extern const float ROBOVIE_MOTOR_DEFAULT_MAX_SPEED;
extern const float ROBOVIE_MOTOR_DEFAULT_ACCEL;

/* ============================= Robovie Action ============================= */
// 3D coordinates
extern const glm::vec3 ROBOVIE_ORG_POS;
extern const glm::vec3 ROBOVIE_ORG_DIR;
extern const glm::vec3 ROBOVIE_ORG_DIR_ARM;
// 3D homogeneous-coordinates
extern const glm::vec4 ROBOVIE_ORG_POS_HM;
extern const glm::vec4 ROBOVIE_ORG_DIR_HM;
extern const glm::vec4 ROBOVIE_ORG_DIR_ARM_HM;
// Parts transformation matrix
extern const glm::mat4 ROBOVIE_TRANSFORM_ORG_TO_BODY;
extern const glm::mat4 ROBOVIE_TRANSFORM_BODY_TO_HEAD;
extern const glm::mat4 ROBOVIE_TRANSFORM_HEAD_TO_LEFT_EYE;
extern const glm::mat4 ROBOVIE_TRANSFORM_HEAD_TO_RIGHT_EYE;
extern const glm::mat4 ROBOVIE_TRANSFORM_BODY_TO_LEFT_ARM;
extern const glm::mat4 ROBOVIE_TRANSFORM_LEFT_ARM_01_TO_23;
extern const glm::mat4 ROBOVIE_TRANSFORM_BODY_TO_RIGHT_ARM;
extern const glm::mat4 ROBOVIE_TRANSFORM_RIGHT_ARM_01_TO_23;

/* ================================ Plotter ================================= */
extern const double PLOTTER_PNG_WAIT_MS;

#endif
