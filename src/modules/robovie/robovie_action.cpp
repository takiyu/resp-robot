#include "robovie_action.h"

#include <math.h>
#include <stdio.h>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/rotate_vector.hpp>

#include "imgui/imgui.h"

namespace {
// X-Y Euler degree (R = Ry * Rx)
inline float XYEulerDegY(const glm::vec3& pos) {
    return glm::degrees(atan2f(pos[0], pos[2]));  // x/z
}

inline float XYEulerDegX(const glm::vec3& pos) {
    float z = sqrtf(pos[0] * pos[0] + pos[2] * pos[2]);
    return glm::degrees(atan2f(z, pos[1]));  // z/y
}

// Z-X Euler degree (R = Rx * Rz)
inline float ZXEulerDegX(const glm::vec3& pos) {
    return glm::degrees(atan2f(pos[2], pos[1]));  // z/y
}

inline float ZXEulerDegZ(const glm::vec3& pos) {
    float y = sqrtf(pos[1] * pos[1] + pos[2] * pos[2]);
    return glm::degrees(atan2f(pos[0], y));  // x/y
}

// Fix degree range
inline float FixDegreeRange(float deg, float center_deg) {
    if (center_deg + 180.f < deg) return deg - 360.f;
    if (deg < center_deg - 180.f) return deg + 360.f;
    return deg;
}

// Rotation matrix
inline glm::mat4 RotX(float deg) {
    return glm::rotate(glm::radians(deg), glm::vec3(1.f, 0.f, 0.f));
}

inline glm::mat4 RotY(float deg) {
    return glm::rotate(glm::radians(deg), glm::vec3(0.f, 1.f, 0.f));
}

inline glm::mat4 RotZ(float deg) {
    return glm::rotate(glm::radians(deg), glm::vec3(0.f, 0.f, 1.f));
}

// X-Y Euler rotation matrix
inline glm::mat4 XYEulerRot(float degy, float degx) {
    return RotY(degy) * RotX(degx);
}

// Z-X Euler rotation matrix
inline glm::mat4 ZXEulerRot(float degx, float degz) {
    return RotX(degx) * RotZ(degz);
}

inline glm::vec3 ExtractTranslation(const glm::mat4& m) {
    return glm::vec3(m[3]);
}

void ConvertDirsWithMatrix(const glm::vec3& org_pos, const glm::vec4& org_dir,
                           const glm::mat4& base_tf_mat,
                           const glm::vec3& dst_pos, glm::vec3& rel_pos_nm,
                           glm::vec3& org_dir_nm) {
    // Transform
    glm::vec3 tf_pos = ExtractTranslation(base_tf_mat) + org_pos;
    glm::vec3 tf_dir = glm::vec3(base_tf_mat * org_dir);
    // Relative position
    glm::vec3 rel_pos = dst_pos - tf_pos;
    // Inverse and set to return
    glm::mat4 mat_inv = glm::inverse(base_tf_mat);
    rel_pos_nm = glm::vec3(mat_inv * glm::vec4(rel_pos, 0));
    org_dir_nm = glm::vec3(org_dir);
}
}

void RobovieAction::initValues() {
    ui_mode = UiMode::NONE;

    // eyelid
    eyelid_per = 0.f;

    // regarding
    body_regard_pos = glm::vec3(0.f, 0.f, 100.f);
    head_regard_pos = glm::vec3(0.f, 0.f, 100.f);
    eyes_regard_pos = glm::vec3(0.f, 0.f, 100.f);
    arm_l01_regard_pos = glm::vec3(10.f, 0.f, 0.f);
    arm_l23_regard_pos = glm::vec3(10.f, 0.f, 0.f);
    arm_r01_regard_pos = glm::vec3(-10.f, 0.f, 0.f);
    arm_r23_regard_pos = glm::vec3(-10.f, 0.f, 0.f);
}

void RobovieAction::updateTransformMatrix() {
    // Current
    float body_deg_y = getMotorCurValue("Body");
    float head_deg_y = getMotorCurValue("Head y");
    float head_deg_x = getMotorCurValue("Head x");
    float eye_l_deg_y = getMotorCurValue("L eye y");
    float eye_l_deg_x = getMotorCurValue("L eye x");
    float eye_r_deg_y = getMotorCurValue("R eye y");
    float eye_r_deg_x = getMotorCurValue("R eye x");
    float arm_l_deg_0 = getMotorCurValue("L arm 0");
    float arm_l_deg_1 = getMotorCurValue("L arm 1");
    float arm_l_deg_2 = getMotorCurValue("L arm 2");
    float arm_l_deg_3 = getMotorCurValue("L arm 3");
    float arm_r_deg_0 = getMotorCurValue("R arm 0");
    float arm_r_deg_1 = getMotorCurValue("R arm 1");
    float arm_r_deg_2 = getMotorCurValue("R arm 2");
    float arm_r_deg_3 = getMotorCurValue("R arm 3");

    // body
    body_base_tf_mat = ROBOVIE_TRANSFORM_ORG_TO_BODY;
    body_tf_mat = body_base_tf_mat * RotY(body_deg_y);
    // head
    head_base_tf_mat = body_tf_mat * ROBOVIE_TRANSFORM_BODY_TO_HEAD;
    head_tf_mat = head_base_tf_mat * XYEulerRot(head_deg_y, head_deg_x);
    // left eye
    eye_l_base_tf_mat = head_tf_mat * ROBOVIE_TRANSFORM_HEAD_TO_LEFT_EYE;
    eye_l_tf_mat = eye_l_base_tf_mat * XYEulerRot(eye_l_deg_y, eye_l_deg_x);
    // right eye
    eye_r_base_tf_mat = head_tf_mat * ROBOVIE_TRANSFORM_HEAD_TO_RIGHT_EYE;
    eye_r_tf_mat = eye_r_base_tf_mat * XYEulerRot(eye_r_deg_y, eye_r_deg_x);
    // left arm 01
    arm_l01_base_tf_mat = body_tf_mat * ROBOVIE_TRANSFORM_BODY_TO_LEFT_ARM;
    arm_l01_tf_mat = arm_l01_base_tf_mat * ZXEulerRot(arm_l_deg_0, arm_l_deg_1);
    // left arm 23
    arm_l23_base_tf_mat = arm_l01_tf_mat * ROBOVIE_TRANSFORM_LEFT_ARM_01_TO_23;
    arm_l23_tf_mat = arm_l23_base_tf_mat * XYEulerRot(arm_l_deg_2, arm_l_deg_3);
    // right arm 01
    arm_r01_base_tf_mat = body_tf_mat * ROBOVIE_TRANSFORM_BODY_TO_RIGHT_ARM;
    arm_r01_tf_mat = arm_r01_base_tf_mat * ZXEulerRot(arm_r_deg_0, arm_r_deg_1);
    // right arm 23
    arm_r23_base_tf_mat = arm_r01_tf_mat * ROBOVIE_TRANSFORM_RIGHT_ARM_01_TO_23;
    arm_r23_tf_mat = arm_r23_base_tf_mat * XYEulerRot(arm_r_deg_2, arm_r_deg_3);

    // TODO: arm_l23_tf_mat and arm_r23_tf_mat is not tested.
}

void RobovieAction::sleepPose() {
    for (int i = 0; i < ROBOVIE_N_MOTOR; i++) {
        setMotor(i, 0.f, 5.0f, 0.05f);  // slowly
    }
    ui_mode = UiMode::SLEEP_POSE;
}

void RobovieAction::moveEyelid(float per, float max_speed, float accel) {
    float deg, v_min, v_max;
    // inverse sign
    float inv_per = 1.f - std::min(std::max(per, 0.f), 1.f);
    // Move left eyelid
    getMotorRange("L eyelid", v_min, v_max);
    deg = (v_max - v_min) * inv_per + v_min;
    setMotor("L eyelid", deg, max_speed, accel);
    // Move right eyelid
    getMotorRange("R eyelid", v_min, v_max);
    deg = (v_max - v_min) * inv_per + v_min;
    setMotor("R eyelid", deg, max_speed, accel);
    // Save for GlUi
    ui_mode = UiMode::EYELID;
    eyelid_per = per;
}

void RobovieAction::regardWithBody(const glm::vec3& pos, float max_speed,
                                   float accel) {
    // Transform vectors
    glm::vec3 rel_pos_nm, org_dir_nm;
    ConvertDirsWithMatrix(ROBOVIE_ORG_POS, ROBOVIE_ORG_DIR_HM,
                          body_base_tf_mat, pos, rel_pos_nm, org_dir_nm);
    // y axis rotation
    float body_deg_y = XYEulerDegY(rel_pos_nm) - XYEulerDegY(org_dir_nm);
    // Set
    setMotor("Body", body_deg_y, max_speed, accel);
    ui_mode = UiMode::REGARD_BODY;
    body_regard_pos = pos;
}

void RobovieAction::regardWithHead(const glm::vec3& pos, float max_speed,
                                   float accel) {
    // Transform vectors
    glm::vec3 rel_pos_nm, org_dir_nm;
    ConvertDirsWithMatrix(ROBOVIE_ORG_POS, ROBOVIE_ORG_DIR_HM,
                          head_base_tf_mat, pos, rel_pos_nm, org_dir_nm);
    // y axis rotation
    float head_deg_y = XYEulerDegY(rel_pos_nm) - XYEulerDegY(org_dir_nm);
    // x axis rotation
    float head_deg_x = XYEulerDegX(rel_pos_nm) - XYEulerDegX(org_dir_nm);
    // Set
    setMotor("Head y", head_deg_y, max_speed, accel);
    setMotor("Head x", head_deg_x, max_speed, accel);
    ui_mode = UiMode::REGARD_HEAD;
    head_regard_pos = pos;
}

void RobovieAction::regardWithEyes(const glm::vec3& pos, float max_speed,
                                   float accel) {
    // Left
    {
        // Transform vectors
        glm::vec3 rel_pos_nm, org_dir_nm;
        ConvertDirsWithMatrix(ROBOVIE_ORG_POS, ROBOVIE_ORG_DIR_HM,
                              eye_l_base_tf_mat, pos, rel_pos_nm, org_dir_nm);
        // y axis rotation
        float eye_l_deg_y = XYEulerDegY(rel_pos_nm) - XYEulerDegY(org_dir_nm);
        // x axis rotation
        float eye_l_deg_x = XYEulerDegX(rel_pos_nm) - XYEulerDegX(org_dir_nm);
        // Set
        setMotor("L eye y", eye_l_deg_y, max_speed, accel);
        setMotor("L eye x", eye_l_deg_x, max_speed, accel);
    }

    // Right
    {
        // Transform vectors
        glm::vec3 rel_pos_nm, org_dir_nm;
        ConvertDirsWithMatrix(ROBOVIE_ORG_POS, ROBOVIE_ORG_DIR_HM,
                              eye_r_base_tf_mat, pos, rel_pos_nm, org_dir_nm);
        // y axis rotation
        float eye_r_deg_y = XYEulerDegY(rel_pos_nm) - XYEulerDegY(org_dir_nm);
        // x axis rotation
        float eye_r_deg_x = XYEulerDegX(rel_pos_nm) - XYEulerDegX(org_dir_nm);
        // Set
        setMotor("R eye y", eye_r_deg_y, max_speed, accel);
        setMotor("R eye x", eye_r_deg_x, max_speed, accel);
    }

    ui_mode = UiMode::REGARD_EYES;
    eyes_regard_pos = pos;
}

void RobovieAction::regardWithArmL01(const glm::vec3& pos, float max_speed,
                                     float accel) {
    // Transform vectors
    glm::vec3 rel_pos_nm, org_dir_nm;
    ConvertDirsWithMatrix(ROBOVIE_ORG_POS, ROBOVIE_ORG_DIR_ARM_HM,
                          arm_l01_base_tf_mat, pos, rel_pos_nm, org_dir_nm);
    // x axis rotation
    float arm_deg_0 = ZXEulerDegX(rel_pos_nm) - ZXEulerDegX(org_dir_nm);
    // z axis rotation
    float arm_deg_1 = ZXEulerDegZ(rel_pos_nm) - ZXEulerDegZ(org_dir_nm);
    // escape skipping > [-180,180]
    arm_deg_0 = FixDegreeRange(arm_deg_0, 0.f);
    arm_deg_1 = FixDegreeRange(arm_deg_1, 0.f);
    // Set
    setMotor("L arm 0", arm_deg_0, max_speed, accel);
    setMotor("L arm 1", arm_deg_1, max_speed, accel);
    ui_mode = UiMode::REGARD_ARM_L01;
    arm_l01_regard_pos = pos;
}

void RobovieAction::regardWithArmL23(const glm::vec3& pos, float max_speed,
                                     float accel) {
    // Transform vectors
    glm::vec3 rel_pos_nm, org_dir_nm;
    ConvertDirsWithMatrix(ROBOVIE_ORG_POS, ROBOVIE_ORG_DIR_ARM_HM,
                          arm_l23_base_tf_mat, pos, rel_pos_nm, org_dir_nm);
    // y axis rotation
    float arm_deg_2 = XYEulerDegY(rel_pos_nm) - XYEulerDegY(org_dir_nm);
    // x axis rotation
    float arm_deg_3 = XYEulerDegX(rel_pos_nm) - XYEulerDegX(org_dir_nm);
    // escape skipping > [-180,180]
    arm_deg_2 = FixDegreeRange(arm_deg_2, 0.f);
    arm_deg_3 = FixDegreeRange(arm_deg_3, 0.f);
    // Set
    setMotor("L arm 2", arm_deg_2, max_speed, accel);
    setMotor("L arm 3", arm_deg_3, max_speed, accel);
    ui_mode = UiMode::REGARD_ARM_L23;
    arm_l23_regard_pos = pos;
}

void RobovieAction::regardWithArmR01(const glm::vec3& pos, float max_speed,
                                     float accel) {
    // Transform vectors
    glm::vec3 rel_pos_nm, org_dir_nm;
    ConvertDirsWithMatrix(ROBOVIE_ORG_POS, ROBOVIE_ORG_DIR_ARM_HM,
                          arm_r01_base_tf_mat, pos, rel_pos_nm, org_dir_nm);
    // x axis rotation
    float arm_deg_0 = ZXEulerDegX(rel_pos_nm) - ZXEulerDegX(org_dir_nm);
    // z axis rotation
    float arm_deg_1 = ZXEulerDegZ(rel_pos_nm) - ZXEulerDegZ(org_dir_nm);
    // escape skipping > [-180,180]
    arm_deg_0 = FixDegreeRange(arm_deg_0, 0.f);
    arm_deg_1 = FixDegreeRange(arm_deg_1, 0.f);
    // Set
    setMotor("R arm 0", arm_deg_0, max_speed, accel);
    setMotor("R arm 1", arm_deg_1, max_speed, accel);
    ui_mode = UiMode::REGARD_ARM_R01;
    arm_r01_regard_pos = pos;
}

void RobovieAction::regardWithArmR23(const glm::vec3& pos, float max_speed,
                                     float accel) {
    // Transform vectors
    glm::vec3 rel_pos_nm, org_dir_nm;
    ConvertDirsWithMatrix(ROBOVIE_ORG_POS, ROBOVIE_ORG_DIR_ARM_HM,
                          arm_r23_base_tf_mat, pos, rel_pos_nm, org_dir_nm);
    // y axis rotation
    float arm_deg_2 = XYEulerDegY(rel_pos_nm) - XYEulerDegY(org_dir_nm);
    // x axis rotation
    float arm_deg_3 = XYEulerDegX(rel_pos_nm) - XYEulerDegX(org_dir_nm);
    // escape skipping > [-180,180]
    arm_deg_2 = FixDegreeRange(arm_deg_2, 0.f);
    arm_deg_3 = FixDegreeRange(arm_deg_3, 0.f);
    // Set
    setMotor("R arm 2", arm_deg_2, max_speed, accel);
    setMotor("R arm 3", arm_deg_3, max_speed, accel);
    ui_mode = UiMode::REGARD_ARM_R23;
    arm_r23_regard_pos = pos;
}

glm::vec3 RobovieAction::getHeadCenterPosition() {
    glm::vec3 pos0 = ExtractTranslation(head_tf_mat);
    glm::vec3 pos1 = ExtractTranslation(eye_l_tf_mat);
    glm::vec3 pos2 = ExtractTranslation(eye_r_tf_mat);
    return (pos0 + pos1 + pos2) / 3.f + ROBOVIE_ORG_POS;
}

glm::vec3 RobovieAction::getEyesCenterPosition() {
    glm::vec3 pos0 = ExtractTranslation(eye_l_tf_mat);
    glm::vec3 pos1 = ExtractTranslation(eye_r_tf_mat);
    return (pos0 + pos1) / 2.f + ROBOVIE_ORG_POS;
}

glm::vec3 RobovieAction::getHeadAngle() {
    glm::vec3 dir = glm::vec3(head_tf_mat * ROBOVIE_ORG_DIR_HM);
    // TODO Set z angle
    return glm::vec3(XYEulerDegX(dir) - XYEulerDegX(ROBOVIE_ORG_DIR),
                     XYEulerDegY(dir) - XYEulerDegY(ROBOVIE_ORG_DIR), 0.f);
}

glm::vec3 RobovieAction::getBodyAngle() {
    glm::vec3 dir = glm::vec3(body_tf_mat * ROBOVIE_ORG_DIR_HM);
    return glm::vec3(0.f, XYEulerDegY(dir) - XYEulerDegY(ROBOVIE_ORG_DIR), 0.f);
}

void RobovieAction::drawGlActionUi() {
    // Common Window
    ImGui::Begin("Modules");
    // Header
    if (ImGui::CollapsingHeader("Robovie Action",
                                ImGuiTreeNodeFlags_DefaultOpen)) {
        // Reset button
        if (ImGui::Button("Reset action values")) {
            initValues();
        }

        // Sleep Pose
        bool ret_sleep =
            ImGui::RadioButton("Sleep Pose  ", &ui_mode, UiMode::SLEEP_POSE);
        ImGui::SameLine();
        ret_sleep |= ImGui::Button("move##sleep");
        if (ret_sleep) this->sleepPose();

        // Eyelid
        bool ret_eyelid =
            ImGui::RadioButton("Eyelid      ", &ui_mode, UiMode::EYELID);
        ImGui::SameLine();
        ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.65f / 3);  // 1/3 width
        ret_eyelid |= ImGui::DragFloat("##eyelid", &eyelid_per);
        if (ret_eyelid) this->moveEyelid(eyelid_per);
        ImGui::PopItemWidth();
        // open / close
        ImGui::SameLine();
        if (ImGui::Button("open##eyelid")) {
            this->openEyelid();
        }
        ImGui::SameLine();
        if (ImGui::Button("close##eyelid")) {
            this->closeEyelid();
        }

        // Body Regard
        bool ret_body =
            ImGui::RadioButton("Body Regard ", &ui_mode, UiMode::REGARD_BODY);
        ImGui::SameLine();
        ret_body |= ImGui::DragFloat3("##bodyregard", &body_regard_pos[0]);
        if (ret_body) this->regardWithBody(body_regard_pos);

        // Head Regard
        bool ret_head =
            ImGui::RadioButton("Head Regard ", &ui_mode, UiMode::REGARD_HEAD);
        ImGui::SameLine();
        ret_head |= ImGui::DragFloat3("##headregard", &head_regard_pos[0]);
        if (ret_head) this->regardWithHead(head_regard_pos);

        // Eyes Regard
        bool ret_eye =
            ImGui::RadioButton("Eyes Regard ", &ui_mode, UiMode::REGARD_EYES);
        ImGui::SameLine();
        ret_eye |= ImGui::DragFloat3("##eyesregard", &eyes_regard_pos[0]);
        if (ret_eye) this->regardWithEyes(eyes_regard_pos);

        // Left Arm 01 Regard
        bool ret_arm_l01 = ImGui::RadioButton("Left arm 01 Regard ", &ui_mode,
                                              UiMode::REGARD_ARM_L01);
        ImGui::SameLine();
        ret_arm_l01 |=
            ImGui::DragFloat3("##arml01regard", &arm_l01_regard_pos[0]);
        if (ret_arm_l01) this->regardWithArmL01(arm_l01_regard_pos);

        // Left Arm 23 Regard
        bool ret_arm_l23 = ImGui::RadioButton("Left arm 23 Regard ", &ui_mode,
                                              UiMode::REGARD_ARM_L23);
        ImGui::SameLine();
        ret_arm_l23 |=
            ImGui::DragFloat3("##arml23regard", &arm_l23_regard_pos[0]);
        if (ret_arm_l23) this->regardWithArmL23(arm_l23_regard_pos);

        // Right Arm 01 Regard
        bool ret_arm_r01 = ImGui::RadioButton("Right arm 01 Regard ", &ui_mode,
                                              UiMode::REGARD_ARM_R01);
        ImGui::SameLine();
        ret_arm_r01 |=
            ImGui::DragFloat3("##armr01regard", &arm_r01_regard_pos[0]);
        if (ret_arm_r01) this->regardWithArmR01(arm_r01_regard_pos);

        // Right Arm 23 Regard
        bool ret_arm_r23 = ImGui::RadioButton("Right arm 23 Regard ", &ui_mode,
                                              UiMode::REGARD_ARM_R23);
        ImGui::SameLine();
        ret_arm_r23 |=
            ImGui::DragFloat3("##armr23regard", &arm_r23_regard_pos[0]);
        if (ret_arm_r23) this->regardWithArmR23(arm_r23_regard_pos);
    }
    ImGui::End();
}

void RobovieAction::updateMotor() {
    updateTransformMatrix();
    RobovieSerial::updateMotor();
}
