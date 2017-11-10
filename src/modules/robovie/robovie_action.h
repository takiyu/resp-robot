#ifndef ROBOVIE_ACTION_160712
#define ROBOVIE_ACTION_160712

#include "robovie_serial.h"

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>

#include "../../config.h"

// Robovie : Action layer class
class RobovieAction : public RobovieSerial {
public:
    RobovieAction(bool debug_log = false)
        : RobovieSerial(true, true, debug_log) {
        initValues();
    }
    ~RobovieAction() {}
    void initValues();

    // Sleep pose
    void sleepPose();
    // Eyelid
    void moveEyelid(float per,
                    float max_speed = ROBOVIE_MOTOR_DEFAULT_MAX_SPEED,
                    float accel = ROBOVIE_MOTOR_DEFAULT_ACCEL);
    void openEyelid() { moveEyelid(1.f); }
    void closeEyelid() { moveEyelid(0.f); }
    // Regarding
    void regardWithBody(const glm::vec3& pos,
                        float max_speed = ROBOVIE_MOTOR_DEFAULT_MAX_SPEED,
                        float accel = ROBOVIE_MOTOR_DEFAULT_ACCEL);
    void regardWithHead(const glm::vec3& pos,
                        float max_speed = ROBOVIE_MOTOR_DEFAULT_MAX_SPEED,
                        float accel = ROBOVIE_MOTOR_DEFAULT_ACCEL);
    void regardWithEyes(const glm::vec3& pos,
                        float max_speed = ROBOVIE_MOTOR_DEFAULT_MAX_SPEED,
                        float accel = ROBOVIE_MOTOR_DEFAULT_ACCEL);
    void regardWithArmL01(const glm::vec3& pos,
                          float max_speed = ROBOVIE_MOTOR_DEFAULT_MAX_SPEED,
                          float accel = ROBOVIE_MOTOR_DEFAULT_ACCEL);
    void regardWithArmL23(const glm::vec3& pos,
                          float max_speed = ROBOVIE_MOTOR_DEFAULT_MAX_SPEED,
                          float accel = ROBOVIE_MOTOR_DEFAULT_ACCEL);
    void regardWithArmR01(const glm::vec3& pos,
                          float max_speed = ROBOVIE_MOTOR_DEFAULT_MAX_SPEED,
                          float accel = ROBOVIE_MOTOR_DEFAULT_ACCEL);
    void regardWithArmR23(const glm::vec3& pos,
                          float max_speed = ROBOVIE_MOTOR_DEFAULT_MAX_SPEED,
                          float accel = ROBOVIE_MOTOR_DEFAULT_ACCEL);

    // Force update
    void updateTransformMatrixForce() { updateTransformMatrix(); }

    // Positions
    glm::vec3 getHeadCenterPosition();
    glm::vec3 getEyesCenterPosition();
    glm::vec3 getHeadAngle();
    glm::vec3 getBodyAngle();

    // Transformation matrix
    glm::mat4 getBodyTfMat() { return body_tf_mat; }
    glm::mat4 getHeadTfMat() { return head_tf_mat; }
    glm::mat4 getArmL01BaseTfMat() { return arm_l01_base_tf_mat; }
    glm::mat4 getArmR01BaseTfMat() { return arm_r01_base_tf_mat; }
    // Eyelid percentage
    float getEyelidPer() { return eyelid_per; }
    // Regarding potions
    glm::vec3 getBodyRegardingPos() { return body_regard_pos; }
    glm::vec3 getHeadRegardingPos() { return head_regard_pos; }
    glm::vec3 getEyesRegardingPos() { return eyes_regard_pos; }
    glm::vec3 getArmL01RegardingPos() { return arm_l01_regard_pos; }
    glm::vec3 getArmL23RegardingPos() { return arm_l23_regard_pos; }
    glm::vec3 getArmR01RegardingPos() { return arm_r01_regard_pos; }
    glm::vec3 getArmR23RegardingPos() { return arm_r23_regard_pos; }

    // UI
    void drawGlActionUi();

private:
    // base transform matrix (the tail)
    glm::mat4 body_base_tf_mat;
    glm::mat4 head_base_tf_mat;
    glm::mat4 eye_l_base_tf_mat, eye_r_base_tf_mat;
    glm::mat4 arm_l01_base_tf_mat;
    glm::mat4 arm_l23_base_tf_mat;
    glm::mat4 arm_r01_base_tf_mat;
    glm::mat4 arm_r23_base_tf_mat;
    // transform matrix (the head)
    glm::mat4 body_tf_mat;
    glm::mat4 head_tf_mat;
    glm::mat4 eye_l_tf_mat, eye_r_tf_mat;
    glm::mat4 arm_l01_tf_mat;
    glm::mat4 arm_l23_tf_mat;
    glm::mat4 arm_r01_tf_mat;
    glm::mat4 arm_r23_tf_mat;

    // Update pose transform matrix (not current but destination pose)
    void updateTransformMatrix();
    // Override of RobovieSerial
    void updateMotor();

    // UI
    enum UiMode {
        NONE,
        SLEEP_POSE,
        EYELID,
        REGARD_BODY,
        REGARD_HEAD,
        REGARD_EYES,
        REGARD_ARM_L01,
        REGARD_ARM_L23,
        REGARD_ARM_R01,
        REGARD_ARM_R23,
    };
    int ui_mode;

    // eyelid
    float eyelid_per;
    // regarding positions
    glm::vec3 body_regard_pos;
    glm::vec3 head_regard_pos;
    glm::vec3 eyes_regard_pos;
    glm::vec3 arm_l01_regard_pos;
    glm::vec3 arm_l23_regard_pos;
    glm::vec3 arm_r01_regard_pos;
    glm::vec3 arm_r23_regard_pos;
};

#endif
