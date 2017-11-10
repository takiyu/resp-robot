#include "sensors.h"

#include <iostream>
// ================================== Sensors ==================================
void Sensors::get(SensorCtr &s) {
    CameraScene &camera_scene = modules->camera_scene;
    RobovieAction &robovie_action = modules->robovie_action;
//     VAD &vad = modules->vad;
//     VoiceRecognition &voice_recog = modules->voice_recog;

    // Face
    camera_scene.getFaces(s.face_valids, s.face_poses, s.att_idxs,
                          s.last_att_idxs, s.last_obj_att_idxs,
                          s.att_back_poses);
    // Objects
    camera_scene.getObjects(s.object_valids, s.object_poses);

//     // VAD
//     s.voice_active = vad.isVoiceActive();
//     s.voice_started = vad.isVoiceStarted();
//     s.voice_finished = vad.isVoiceFinished();
//     s.voice_len_ms = vad.getVoiceLengthMs();
//
//     // Voice Recognition
//     s.voice_recog_processing = voice_recog.isProcessing();
//     s.voice_recoged = voice_recog.hasResult();
//     if (s.voice_recoged) {
//         s.voice_recog_result = voice_recog.popResult();
//     } else {
//         s.voice_recog_result = "";
//     }

    // Robovie
    s.robovie_body_tf_mat = robovie_action.getBodyTfMat();
    s.robovie_head_tf_mat = robovie_action.getHeadTfMat();
    s.robovie_arm_l01_base_tf_mat = robovie_action.getArmL01BaseTfMat();
    s.robovie_arm_r01_base_tf_mat = robovie_action.getArmR01BaseTfMat();
    s.robovie_head_pos = robovie_action.getHeadCenterPosition();
    s.robovie_eyes_pos = robovie_action.getEyesCenterPosition();
    s.robovie_eyelid_per = robovie_action.getEyelidPer();
    s.robovie_body_regard_pos = robovie_action.getBodyRegardingPos();
    s.robovie_head_regard_pos = robovie_action.getHeadRegardingPos();
    s.robovie_eyes_regard_pos = robovie_action.getEyesRegardingPos();
    s.robovie_left_arm_regard_poses =
        std::pair<glm::vec3, glm::vec3>(robovie_action.getArmL01RegardingPos(),
                                        robovie_action.getArmL23RegardingPos());
    s.robovie_right_arm_regard_poses =
        std::pair<glm::vec3, glm::vec3>(robovie_action.getArmR01RegardingPos(),
                                        robovie_action.getArmR23RegardingPos());
}
