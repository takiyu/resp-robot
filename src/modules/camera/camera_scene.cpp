#include "camera_scene.h"

#include <cmath>

#include <glm/gtc/constants.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "imgui/imgui.h"

#include "../../viewer/render/gl_utils.h"

namespace {
inline glm::vec3 CastVec(const cv::Point3f& src) {
    return glm::vec3(src.x, src.y, src.z);
}

inline glm::vec3 CvtColor(const cv::Scalar& src) {
    return glm::vec3(src[2], src[1], src[0]) / 255.f;
}

inline cv::Point3f CastVec(const glm::vec3& src) {
    return cv::Point3f(src.x, src.y, src.z);
}

void EmitRay(const cv::Point3f& ray_org, const cv::Point3f& ray_dir,
             const cv::Point3f& obj_position, float& depth, float& distance) {
    // Implementation of "Distance from a point to a line"
    //   https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line

    //  norm(ray_dir) must be 1
    cv::Point3f a_p = ray_org - obj_position;
    depth = -a_p.dot(ray_dir);
    distance = cv::norm(a_p + depth * ray_dir);
}

float AttentionIntensity(float depth, float distance) {
    // rates [0:1]
    float depth_rate = depth / ATTENTION_MAX_DEPTH;
    float theta_rate = atan2f(distance, depth);
    theta_rate *= (180.f / glm::pi<float>() / ATTENTION_MAX_DEGREE);
    // check ranges
    if (depth_rate <= 0.f || 1.f < depth_rate) return 0.f;
    if (1.f < theta_rate) return 0.f;
    // error [0:1]
    float error = ATTENTION_ERROR_COEFF_THETA * theta_rate +
                  ATTENTION_ERROR_COEFF_DEPTH * depth;
    error /= (ATTENTION_ERROR_COEFF_THETA + ATTENTION_ERROR_COEFF_DEPTH);
    // intensity [0:1]
    float intensity = 1.f / error;
    return intensity;
}
}

namespace {
void UpdateScene(
    WebCamera& webcam, FaceEstimator& face_estimator,
    ObjectDetector& object_detector, const glm::vec3& robovie_head_position,
    const glm::vec3& robovie_head_angle, std::vector<char>& face_estimated,
    std::vector<char>& gaze_estimated, std::vector<cv::Point3f>& face_angles,
    std::vector<cv::Point3f>& face_positions,
    std::vector<cv::Point3f>& gaze_orgs, std::vector<cv::Point3f>& gaze_dirs,
    std::vector<std::vector<cv::Rect> >& object_bboxes,
    std::vector<std::vector<cv::Point3f> >& object_positions,
    std::vector<CameraScene::AttentionIdx>& attention_idxs,
    std::vector<float>& attention_depths,
    std::vector<glm::vec3>& attention_back_poses, cv::Mat& debug_frame) {
    // === Capture webcam ===
    cv::Mat frame = webcam.capture();
    if (frame.empty()) return;
    CV_DbgAssert(frame.type() == CV_8UC3);
    cv::Mat gray;
    cv::cvtColor(frame, gray, CV_BGR2GRAY);
    debug_frame = frame.clone();

    // === Face estimator ===
    face_estimator.estimate(gray);
    face_estimator.getFaceResults(face_estimated, face_angles, face_positions);
    face_estimator.getGazeResults(gaze_estimated, gaze_orgs, gaze_dirs);
    face_estimator.drawDebug(debug_frame);  // debug draw

    // === Object detector ===
    object_detector.detect(frame, object_bboxes, object_positions);
    object_detector.drawDebug(debug_frame, object_bboxes);  // debug draw

    // === Attention estimation ===
    int n_gaze = gaze_estimated.size();
    attention_idxs.resize(n_gaze);
    attention_depths.resize(n_gaze);
    attention_back_poses.resize(n_gaze);
    for (int i = 0; i < n_gaze; i++) {
        if (!gaze_estimated[i]) {
            attention_idxs[i] = CameraScene::AttentionIdx::NONE;
            continue;
        }

        float max_att_intensity = 0.f;
        CameraScene::AttentionIdx max_att_idx = CameraScene::AttentionIdx::NONE;
        float max_att_depth = ATTENTION_MAX_DEPTH;
        float depth, distance;
        glm::vec3 back_pos(0.f);
        // Robovie
        {
            EmitRay(gaze_orgs[i], gaze_dirs[i], CastVec(robovie_head_position),
                    depth, distance);
            float att_intensity = AttentionIntensity(depth, distance);
            if (max_att_intensity < att_intensity) {
                max_att_intensity = att_intensity;
                max_att_depth = depth;
                max_att_idx = CameraScene::AttentionIdx::ROBOVIE;
            }
        }
        // Objects
        for (int label = 0; label < N_OBJECT_DETECT; label++) {
            for (int i = 0; i < object_positions[label].size(); i++) {
                EmitRay(gaze_orgs[i], gaze_dirs[i], object_positions[label][i],
                        depth, distance);
                float att_intensity = AttentionIntensity(depth, distance);
                if (max_att_intensity < att_intensity) {
                    max_att_intensity = att_intensity;
                    max_att_depth = depth;
                    max_att_idx = static_cast<CameraScene::AttentionIdx>(
                        CameraScene::AttentionIdx::OBJECT + label);
                }
            }
        }
        // Back position
        if (max_att_idx == CameraScene::AttentionIdx::NONE) {
            back_pos = CastVec(gaze_dirs[i]) * ATTENTION_BACK_DEPTH +
                       CastVec(gaze_orgs[i]);
            max_att_depth = ATTENTION_BACK_DEPTH;
        }
        // Register
        attention_idxs[i] = max_att_idx;
        attention_depths[i] = max_att_depth;
        attention_back_poses[i] = back_pos;
    }
}

void DrawGlUi(GLMesh& face_mesh, GLLine& gaze_line, GLMesh& robovie_mesh,
              std::vector<GLMesh>& object_meshes,
              const std::vector<char>& face_estimated,
              const std::vector<char>& gaze_estimated,
              const std::vector<cv::Point3f>& face_angles,
              const std::vector<cv::Point3f>& face_positions,
              const std::vector<cv::Point3f>& gaze_orgs,
              const std::vector<cv::Point3f>& gaze_dirs,
              const std::vector<std::vector<cv::Point3f> >& object_positions,
              const std::vector<CameraScene::AttentionIdx>& attention_idxs,
              const std::vector<float>& attention_depths,
              const glm::vec3& robovie_head_position,
              const glm::vec3& robovie_head_angle, const cv::Mat& debug_frame,
              const GLuint debug_frame_tex_id, GLsizei& debug_frame_width,
              GLsizei& debug_frame_height, int fps) {
    // === Shapes ===
    // Update face mesh
    for (int i = 0; i < face_estimated.size(); i++) {
        if (face_estimated[i]) {
            // Set position
            face_mesh.setVisibility(true);
            face_mesh.setTranslation(CastVec(face_positions[i]));
            face_mesh.setAngle(CastVec(face_angles[i]));
            // Draw
            face_mesh.draw();
        }
    }
    // Update gaze line
    for (int i = 0; i < gaze_estimated.size(); i++) {
        if (gaze_estimated[i]) {
            // Set position
            gaze_line.setVisibility(true);
            gaze_line.setTailPosition(CastVec(gaze_orgs[i]));
            gaze_line.setHeadPosition(
                CastVec(gaze_orgs[i] + gaze_dirs[i] * attention_depths[i]));
            // Draw
            gaze_line.draw();
        }
    }
    // Update Robovie mesh
    {
        robovie_mesh.setVisibility(true);
        robovie_mesh.setTranslation(robovie_head_position);
        robovie_mesh.setAngle(robovie_head_angle);
        cv::Scalar color = ROBOVIE_VIEW_COLOR;
        bool is_attended = false;
        for (int i = 0; i < attention_idxs.size(); i++) {
            if (attention_idxs[i] == CameraScene::AttentionIdx::ROBOVIE) {
                is_attended = true;
                break;
            }
        }
        if (!is_attended) color /= VIEW_ATTENSION_COLOR_SCALE;
        robovie_mesh.setColor(CvtColor(color));
        // Draw
        robovie_mesh.draw();
    }
    // Update object meshes
    for (int label = 0; label < object_positions.size(); label++) {
        if (object_positions[label].size() > 0) {
            // Set visibility position, and rotation
            object_meshes[label].setVisibility(true);
            object_meshes[label].setTranslation(
                CastVec(object_positions[label][0]));
            // (I suppose that the object is not rotated, so rotation is same to
            //  the camera)  TODO: Consider dynamic angles
            object_meshes[label].setAngle(CastVec(WEBCAMERA_ANGLES));
            // Set color
            cv::Scalar color = OBJECT_VIEW_COLORS[label];
            bool is_attended = false;
            for (int i = 0; i < attention_idxs.size(); i++) {
                if ((attention_idxs[i] - CameraScene::AttentionIdx::OBJECT) ==
                    label) {
                    is_attended = true;
                    break;
                }
            }
            if (!is_attended) color /= VIEW_ATTENSION_COLOR_SCALE;
            object_meshes[label].setColor(CvtColor(color));
            // Draw
            object_meshes[label].draw();
        } else {
            object_meshes[label].setVisibility(false);
        }
    }

    // === Debug frame texture (ImGui) ===
    if (!debug_frame.empty()) {
        // Update texture
        glBindTexture(GL_TEXTURE_2D, debug_frame_tex_id);
        if (debug_frame.rows == debug_frame_height &&
            debug_frame.cols == debug_frame_width) {
            // update data only
            // format:BGR
            glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, debug_frame_width,
                            debug_frame_height, GL_BGR, GL_UNSIGNED_BYTE,
                            debug_frame.data);
        } else {
            // update texture size and data
            debug_frame_height = debug_frame.rows;
            debug_frame_width = debug_frame.cols;
            // internal_format:RGB, format:BGR
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, debug_frame_width,
                         debug_frame_height, 0, GL_BGR, GL_UNSIGNED_BYTE,
                         debug_frame.data);
        }
        checkGlError();

        // Draw (Independent Window)
        ImGui::Begin("Camera Scene", NULL, ImGuiWindowFlags_AlwaysAutoResize);
        // image
        ImTextureID imgui_tex_id = (void*)(intptr_t)debug_frame_tex_id;
        ImGui::Image(imgui_tex_id,
                     ImVec2(debug_frame_width, debug_frame_height),
                     ImVec2(0, 0), ImVec2(1, 1), ImColor(255, 255, 255, 255),
                     ImColor(0, 0, 0, 0));
        // fps text
        std::stringstream fps_ss;
        fps_ss << "fps: " << fps;
        ImGui::Text("%s", fps_ss.str().c_str());
        ImGui::End();
    }
}
}  // namespace

void CameraScene::init() {
    printf("* Initialize camera scene\n");
    webcam.open();
    face_estimator.init();
    object_detector.init();

    // Start scene update thread
    prev_clock = clock();
    update_thread = new boost::thread([&]() {
        printf("* Start a camera scene updating thread\n");
        while (true) {
            // Fix FPS
            clock_t curr_clock = clock();
            double elapse = (double)(curr_clock - prev_clock) / CLOCKS_PER_SEC;
            double sleep_ms = ((double)(1.f / MAX_FPS) - elapse) * 1000.0;
            prev_clock = curr_clock;
            if (sleep_ms > 0) {
                auto boost_sleep_ms = boost::posix_time::milliseconds(sleep_ms);
                boost::this_thread::sleep(boost_sleep_ms);
            } else {
                boost::this_thread::interruption_point();
            }
            // Update
            this->updateScene();
        }
    });
}

void CameraScene::exit() {
    if (update_thread != NULL) {
        printf("* Exit a camera scene updating thread\n");
        update_thread->interrupt();
        update_thread->join();
        delete update_thread;
        update_thread = NULL;
    }
    printf("* Exit camera scene\n");
    webcam.close();
    attention_idxs.clear();
    last_attention_idxs.clear();
    last_obj_attention_idxs.clear();
    attention_back_poses.clear();
}

void CameraScene::getFaces(std::vector<char>& valids,
                           std::vector<glm::vec3>& poses,
                           std::vector<AttentionIdx>& att_idxs,
                           std::vector<AttentionIdx>& last_att_idxs,
                           std::vector<AttentionIdx>& last_obj_att_idxs,
                           std::vector<glm::vec3>& att_back_poses) {
    // Copy shared variables
    m_update.lock();  // lock
    valids = this->face_estimated;
    std::vector<cv::Point3f> poses_cv = this->face_positions;
    att_idxs = this->attention_idxs;
    last_att_idxs = this->last_attention_idxs;
    last_obj_att_idxs = this->last_obj_attention_idxs;
    att_back_poses = this->attention_back_poses;
    m_update.unlock();  // unlock

    // Convert types
    poses.resize(poses_cv.size());
    for (int i = 0; i < poses.size(); i++) {
        poses[i] = CastVec(poses_cv[i]);
    }
}

void CameraScene::getObjects(std::vector<char>& valids,
                             std::vector<glm::vec3>& poses) {
    // Copy shared variables
    m_update.lock();  // lock
    std::vector<std::vector<cv::Point3f> > poses_cv = this->object_positions;
    m_update.unlock();  // unlock

    // Singulate and convert types
    int n = poses_cv.size();
    valids.resize(n);
    poses.resize(n);
    for (int i = 0; i < n; i++) {
        valids[i] = (poses_cv[i].size() > 0);
        if (valids[i]) {
            poses[i] = CastVec(poses_cv[i][0]);
        } else {
            poses[i] = glm::vec3(0.f);
        }
    }
}

void CameraScene::setRobovieStatus(const glm::vec3& head_pos,
                                   const glm::vec3& head_angle) {
    // Copy to shared variables
    m_update.lock();  // lock
    robovie_head_position = head_pos;
    robovie_head_angle = head_angle;
    m_update.unlock();  // unlock
}

void CameraScene::initGlUi() {
    // === Shapes ===
    // face shape
    face_mesh.loadObjFile(MESH_FACE_PATH, 15.f);
    // gaze lines
    gaze_line.setColor(glm::vec3(0.f, 0.f, 1.f));
    // Robovie shape
    robovie_mesh.loadObjFile(MESH_ROBOVIE_PATH, 12.f);
    // object shapes
    object_meshes.resize(N_OBJECT_DETECT);
    for (int i = 0; i < N_OBJECT_DETECT; i++) {
        const float& WIDTH = OBJECT_3D_SIZES[i][0];
        object_meshes[i].loadObjFile(MESH_OBJECT_PATHS[i], WIDTH);
    }

    {
        // === Debug frame texture ===
        boost::mutex::scoped_lock lock(m_update);  // lock
        if (debug_frame.empty()) {
            // Create dummy image
            debug_frame =
                cv::Mat::zeros(debug_frame_height, debug_frame_width, CV_8UC3) +
                cv::Scalar(0, 255, 0);  // green
        } else {
            debug_frame_height = debug_frame.rows;
            debug_frame_width = debug_frame.cols;
        }
        // Create texture
        glGenTextures(1, &debug_frame_tex_id);
        glBindTexture(GL_TEXTURE_2D, debug_frame_tex_id);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        // internal_format:RGB, format:BGR
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, debug_frame_width,
                     debug_frame_height, 0, GL_BGR, GL_UNSIGNED_BYTE,
                     debug_frame.data);
        checkGlError();
    }
}

void CameraScene::drawGlUi() {
    // Copy shared variables
    m_update.lock();  // lock
    const std::vector<char> face_estimated = this->face_estimated;
    const std::vector<char> gaze_estimated = this->gaze_estimated;
    const std::vector<cv::Point3f> face_angles = this->face_angles;
    const std::vector<cv::Point3f> face_positions = this->face_positions;
    const std::vector<cv::Point3f> gaze_orgs = this->gaze_orgs;
    const std::vector<cv::Point3f> gaze_dirs = this->gaze_dirs;
    const std::vector<std::vector<cv::Point3f> > object_positions =
        this->object_positions;
    const std::vector<AttentionIdx> attention_idxs = this->attention_idxs;
    const std::vector<float> attention_depths = this->attention_depths;
    const glm::vec3 robovie_head_position = this->robovie_head_position;
    const glm::vec3 robovie_head_angle = this->robovie_head_angle;
    const cv::Mat debug_frame = this->debug_frame.clone();
    const int fps = this->fps.getFps();
    m_update.unlock();  // unlock

    DrawGlUi(this->face_mesh, this->gaze_line, this->robovie_mesh,
             this->object_meshes, face_estimated, gaze_estimated, face_angles,
             face_positions, gaze_orgs, gaze_dirs, object_positions,
             attention_idxs, attention_depths, robovie_head_position,
             robovie_head_angle, debug_frame, this->debug_frame_tex_id,
             this->debug_frame_width, this->debug_frame_height, fps);
}

void CameraScene::showDebugFrame() {
    m_update.lock();  // lock
    if (!debug_frame.empty()) {
        cv::imshow("camera scene debug", debug_frame);
    }
    m_update.unlock();  // unlock
}

void CameraScene::updateScene() {
    // === Local variables ===
    // Copy to local variables
    m_update.lock();     // lock
    this->fps.update();  // fps update
    const glm::vec3 robovie_head_position = this->robovie_head_position;
    const glm::vec3 robovie_head_angle = this->robovie_head_angle;
    std::vector<AttentionIdx> last_attention_idxs = this->last_attention_idxs;
    std::vector<AttentionIdx> last_obj_attention_idxs =
        this->last_obj_attention_idxs;
    m_update.unlock();  // unlock

    // Declare empty local variables
    std::vector<char> face_estimated, gaze_estimated;
    std::vector<cv::Point3f> face_angles, face_positions;
    std::vector<cv::Point3f> gaze_orgs, gaze_dirs;
    std::vector<std::vector<cv::Rect> > object_bboxes;
    std::vector<std::vector<cv::Point3f> > object_positions;
    std::vector<CameraScene::AttentionIdx> attention_idxs;
    std::vector<float> attention_depths;
    std::vector<glm::vec3> attention_back_poses;
    cv::Mat debug_frame;

    // === Update scene ===
    UpdateScene(this->webcam, this->face_estimator, this->object_detector,
                robovie_head_position, robovie_head_angle, face_estimated,
                gaze_estimated, face_angles, face_positions, gaze_orgs,
                gaze_dirs, object_bboxes, object_positions, attention_idxs,
                attention_depths, attention_back_poses, debug_frame);

    // === Update attention history ===
    int n_gaze = attention_idxs.size();
    last_attention_idxs.resize(n_gaze);
    last_obj_attention_idxs.resize(n_gaze);
    for (int i = 0; i < n_gaze; i++) {
        // Attention history
        if (attention_idxs[i] != AttentionIdx::NONE) {
            last_attention_idxs[i] = attention_idxs[i];
            if (attention_idxs[i] >= AttentionIdx::OBJECT) {
                last_obj_attention_idxs[i] = attention_idxs[i];
            }
        }
    }

    // === Set the results ===
    // Writing operation is only here.
    m_update.lock();  // lock
    this->face_estimated = face_estimated;
    this->gaze_estimated = gaze_estimated;
    this->face_angles = face_angles;
    this->face_positions = face_positions;
    this->gaze_orgs = gaze_orgs;
    this->gaze_dirs = gaze_dirs;
    this->object_bboxes = object_bboxes;
    this->object_positions = object_positions;
    this->attention_idxs = attention_idxs;
    this->attention_depths = attention_depths;
    this->debug_frame = debug_frame;
    this->last_attention_idxs = last_attention_idxs;
    this->last_obj_attention_idxs = last_obj_attention_idxs;
    this->attention_back_poses = attention_back_poses;
    m_update.unlock();  // unlock
}
