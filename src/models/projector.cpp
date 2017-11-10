#include "projector.h"

#include <string>

namespace {
template <typename T>
float DecideMaxSpeed(const MovementCtr::BasePart<T> &part, float max_speed) {
    if (part.max_speed > 0.f) {
        return part.max_speed;
    }
    return max_speed;
}

template <typename T>
float DecideAccel(const MovementCtr::BasePart<T> &part, float accel) {
    if (part.accel > 0.f) {
        return part.accel;
    }
    return accel;
}
}

void RobovieProjector::project(const MovementCtr &result_behavior) {
    RobovieAction &robovie = modules->robovie_action;
    const MovementCtr &b = result_behavior;

    // body
    if (!b.body.empty && !b.body.used) {
        float max_speed = DecideMaxSpeed(b.body, MAX_SPEED);
        float accel = DecideAccel(b.body, ACCEL);
        robovie.regardWithBody(b.body.v, max_speed, accel);
    }

    robovie.updateTransformMatrixForce();  // TODO Use only changed

    // head
    if (!b.head.empty && !b.head.used) {
        float max_speed = DecideMaxSpeed(b.head, MAX_SPEED);
        float accel = DecideAccel(b.head, ACCEL);
        robovie.regardWithHead(b.head.v, max_speed, accel);
    }

    robovie.updateTransformMatrixForce();  // TODO Use only changed

    // eyes
    if (!b.eyes.empty && !b.eyes.used) {
        float max_speed = DecideMaxSpeed(b.eyes, MAX_SPEED);
        float accel = DecideAccel(b.eyes, ACCEL);
        robovie.regardWithEyes(b.eyes.v, max_speed, accel);
    }

    robovie.updateTransformMatrixForce();  // TODO Use only changed

    // eyelid
    if (!b.eyelid.empty && !b.eyelid.used) {
        float max_speed = DecideMaxSpeed(b.eyelid, MAX_SPEED);
        float accel = DecideAccel(b.eyelid, ACCEL);
        robovie.moveEyelid(b.eyelid.v, max_speed, accel);
    }

    // right_arm
    if (!b.right_arm.empty && !b.right_arm.used) {
        float max_speed = DecideMaxSpeed(b.right_arm, MAX_SPEED);
        float accel = DecideAccel(b.right_arm, ACCEL);
        robovie.regardWithArmR01(b.right_arm.v.first, max_speed, accel);
        robovie.regardWithArmR23(b.right_arm.v.second, max_speed, accel);
    }

    // left_arm
    if (!b.left_arm.empty && !b.left_arm.used) {
        float max_speed = DecideMaxSpeed(b.left_arm, MAX_SPEED);
        float accel = DecideAccel(b.left_arm, ACCEL);
        robovie.regardWithArmL01(b.left_arm.v.first, max_speed, accel);
        robovie.regardWithArmL23(b.left_arm.v.second, max_speed, accel);
    }

    // voice
    if (!b.voice.empty && !b.voice.used) {
//         modules->voice_synthesis.synthesis(b.voice.v, modules->wav_player,
//                                            false);  // non-blocking
    }
}
