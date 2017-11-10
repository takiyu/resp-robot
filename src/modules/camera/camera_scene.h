#ifndef CAMERA_SCENE_160801
#define CAMERA_SCENE_160801

#include <boost/thread.hpp>
#include <opencv2/core/core.hpp>
#include <string>
#include <vector>

#include <GL/glew.h>
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>

#include "../../fps.h"
#include "../../viewer/gl_shapes.h"
#include "camera.h"
#include "face_estimator.h"
#include "object_detector.h"

#include "../../config.h"

class CameraScene {
public:
    CameraScene()
        :  // dummy initial size > 0
          debug_frame_width(100),
          debug_frame_height(100),
          update_thread(NULL),
          MAX_FPS(WEBCAMERA_FPS) {}
    ~CameraScene() {}

    void init();
    void exit();

    // Attention indices
    //   OBJECT + i -> i-th object
    enum AttentionIdx { NONE = 0, ROBOVIE, OBJECT };

    void getFaces(std::vector<char>& valids, std::vector<glm::vec3>& poses,
                  std::vector<AttentionIdx>& att_idxs,
                  std::vector<AttentionIdx>& last_att_idxs,
                  std::vector<AttentionIdx>& last_obj_att_idxs,
                  std::vector<glm::vec3>& att_back_poses);
    void getObjects(std::vector<char>& valids, std::vector<glm::vec3>& poses);

    // Set robovie status for drawing mesh
    void setRobovieStatus(const glm::vec3& head_pos,
                          const glm::vec3& head_angle);

    // GL UIs
    void initGlUi();
    void drawGlUi();

    // CV frame
    void showDebugFrame();

private:
    boost::mutex m_update;

    // Modules
    WebCamera webcam;
    FaceEstimator face_estimator;
    ObjectDetector object_detector;

    // Face Estimator results
    std::vector<char> face_estimated, gaze_estimated;
    std::vector<cv::Point3f> face_angles, face_positions;
    std::vector<cv::Point3f> gaze_orgs, gaze_dirs;

    // Object Detector results
    std::vector<std::vector<cv::Rect> > object_bboxes;
    std::vector<std::vector<cv::Point3f> > object_positions;

    // Attentions
    std::vector<AttentionIdx> attention_idxs, last_attention_idxs,
        last_obj_attention_idxs;
    std::vector<float> attention_depths;
    std::vector<glm::vec3> attention_back_poses;

    // Robovie status which are fetched from RobovieAction
    glm::vec3 robovie_head_position;
    glm::vec3 robovie_head_angle;

    // Fps for GlUi
    FpsCounter fps;

    // Viewer shapes
    GLMesh face_mesh;
    GLLine gaze_line;
    GLMesh robovie_mesh;
    std::vector<GLMesh> object_meshes;

    // Debug frame
    cv::Mat debug_frame;
    GLuint debug_frame_tex_id;
    GLsizei debug_frame_width, debug_frame_height;

    // Thread
    const int MAX_FPS;
    clock_t prev_clock;
    void updateScene();
    boost::thread* update_thread;
};

#endif
