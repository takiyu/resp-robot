#include "modules.h"

#include "imgui/imgui.h"

void Modules::init() {
    if (inited) return;
    printf("* Initialize modules\n");

    // Camera scene
    camera_scene.init();
    // Robovie
    robovie_action.open();

//     // Port Audio
//     InitializePortAudio();
//     // VAD
//     vad.init();
//     // Voice recognition
//     voice_recog.init();
//     // Voice synthesis
//     voice_synthesis.init();

    inited = true;
}

void Modules::exit() {
    if (!inited) return;
    printf("* Exit modules\n");

    // Camera scene
    camera_scene.exit();
    // Robovie
    robovie_action.close();

//     // VAD
//     vad.exit();
//     // Voice recognition
//     voice_recog.exit();
//     // Voice synthesis
//     voice_synthesis.exit();
//     // Port Audio
//     TerminatePortAudio();

    inited = false;
}

void Modules::update() {
    // Set Robovie head status to the scene
    camera_scene.setRobovieStatus(robovie_action.getHeadCenterPosition(),
                                  robovie_action.getHeadAngle());
    // Dynamic webcamera
    glm::vec3 body_angle = robovie_action.getBodyAngle();
    cv::Point3f body_angle_cv(body_angle.x, body_angle.y, body_angle.z);
    setWebCameraDynamicAngle(body_angle_cv);
    // Update VAD status
//     vad.updateStatus();
}

void Modules::initGlUi() {
    // Initialize shapes
    camera_scene.initGlUi();
}

void Modules::drawGlUi() {
    // Draw shapes and Camera frame with ImGui
    camera_scene.drawGlUi();

    // Modules window
    ImGui::Begin("Modules");
    robovie_action.drawGlMotorUi();
    robovie_action.drawGlActionUi();
//     vad.drawGlUi();
//     voice_recog.drawGlUi();
//     voice_synthesis.drawGlUi();
    ImGui::End();
}
