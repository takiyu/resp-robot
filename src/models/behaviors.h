#ifndef BEHAVIORS_160914
#define BEHAVIORS_160914

#include <chrono>

#include "behavior_base.h"

// ===== Sleep =====
class SleepBehavior : public BaseBehavior {
public:
    SleepBehavior() : BaseBehavior("Sleep", true) {}

    // Static parameters (speed is common now)
    const float SPEED = 1.f, ACCEL = 0.01f, WAIT_MS = 100.f;

private:
    void declareWeights();
    bool shouldOccur();
    void behave();
};

// ===== Move head =====
class MoveHeadBehavior : public BaseBehavior {
public:
    enum Type { UP, DOWN, RIGHT, LEFT };
    const Type TYPE;

    MoveHeadBehavior(Type type) :
        TYPE(type), BaseBehavior(selectTag(type), false) {}

    // Type selection (tag)
    std::string selectTag(Type type) {
        switch (type) {
            case Type::UP: return "MoveHead[Up]";
            case Type::DOWN: return "MoveHead[Down]";
            case Type::RIGHT: return "MoveHead[Right]";
            case Type::LEFT: return "MoveHead[Left]";
        }
        return "MoveHead[?]";
    }

    // Static parameters
    const float SPEED = 0.5f, ACCEL = 0.10f, WAIT_MS = 200.f;
    const float DEG = 10.0f;

private:
    void declareWeights();
    bool shouldOccur();
    void behave();
};

// ===== Blink =====
class BlinkBehavior : public BaseBehavior {
public:
    enum Type { RAND, DYNAMIC };
    const Type TYPE;

    BlinkBehavior(Type type)
        : TYPE(type), BaseBehavior(selectTag(type), selectAutoOccuring(type)) {}

    // Type selection (tag)
    std::string selectTag(Type type) {
        switch (type) {
            case Type::RAND: return "Blink[rand]";
            case Type::DYNAMIC: return "Blink[dynamic]";
        }
        return "Blink[?]";
    }

    // Type selection (auto_occurring)
    bool selectAutoOccuring(Type type) {
        switch (type) {
            case Type::RAND: return true;
            case Type::DYNAMIC: return true;
        }
        return false;
    }

    // Static parameters
    const float SPEED = 8.f, ACCEL = 5.f, WAIT_MS = 500.f;
    const float PER = 1.f;
    const int PROB_DENOM = 500;  // rand
    const float THRESHOLD_DEG = 40.f; // dynamic

    // For Type::DYNAMIC
    glm::vec3 prev_eyes_regard_pos;
private:
    void declareWeights();
    bool shouldOccur();
    void behave();
};

// ===== Nod =====
// `http://www.oit.ac.jp/is/~koda/hiserver01/labmember/wakisaka/public_html/
// defence_wakisaka.pdf`
class NodBehavior : public BaseBehavior {
public:
    NodBehavior() : BaseBehavior("Nod", false) {}

    // Static parameters
    const float SPEED = 1.5f, ACCEL = 0.40f, WAIT_MS = 260.f;
    const float DEG = 40.f;

private:
    void declareWeights();
    bool shouldOccur();
    void behave();
};

// ===== HandWave =====
class HandWaveBehavior : public BaseBehavior {
public:
    enum Type { LEFT_RAND, RIGHT_RAND, BOTH_RAND,
                LEFT_RESP, RIGHT_RESP, BOTH_RESP };
    const Type TYPE;

    HandWaveBehavior(Type type)
        : TYPE(type), BaseBehavior(selectTag(type), selectAutoOccuring(type)) {}

    // Type selection (tag)
    std::string selectTag(Type type) {
        switch (type) {
            case Type::LEFT_RAND: return "HandWave[LeftRandom]";
            case Type::RIGHT_RAND: return "HandWave[RightRandom]";
            case Type::BOTH_RAND: return "HandWave[BothRandom]";
            case Type::LEFT_RESP: return "HandWave[LeftResponse]";
            case Type::RIGHT_RESP: return "HandWave[RightResoponse]";
            case Type::BOTH_RESP: return "HandWave[BothResponse]";
        }
        return "HandWave[?]";
    }

    // Type selection (auto_occurring)
    bool selectAutoOccuring(Type type) {
        switch (type) {
            case Type::LEFT_RAND: return false;
            case Type::RIGHT_RAND: return false;
            case Type::BOTH_RAND: return false;
            case Type::LEFT_RESP: return false;
            case Type::RIGHT_RESP: return false;
            case Type::BOTH_RESP: return false;
        }
        return false;
    }

    // Static parameters
    const float SPEED = 3.0f, ACCEL = 0.5f, WAIT_MS = 600.f;
    const int N_WAVE = 10, N_WAVE_AUTO = 2;
    const int PROB_DENOM = (TYPE == Type::BOTH_RAND ? 4000 : 2000);

private:
    void declareWeights();
    bool shouldOccur();
    void behave();
};

// ===== Hand Holding out =====
class HandHoldOutBehavior : public BaseBehavior {
public:
    enum Type { RIGHT, LEFT, BOTH };
    const Type TYPE;

    HandHoldOutBehavior(Type type) :
        TYPE(type), BaseBehavior(selectTag(type), false) {}

    // Type selection (tag)
    std::string selectTag(Type type) {
        switch (type) {
            case Type::RIGHT: return "HandHoldingOut[Right]";
            case Type::LEFT: return "HandHoldingOut[Left]";
            case Type::BOTH: return "HandHoldingOut[Both]";
        }
        return "HandHoldingOut[?]";
    }

    // Static parameters
    const float SPEED = 0.5f, ACCEL = 0.10f, WAIT_MS = 200.f;
    const float DEG = 10.0f;

private:
    void declareWeights();
    bool shouldOccur();
    void behave();
};

// ===== Lookat =====
class LookatBehavior : public BaseBehavior {
public:
    enum Type {
        RAND_FACE,
        RAND_OBJ,
        CLOSEST_FACE,
        CLOSEST_OBJ,
        MOVING_FACE,
        MOVING_OBJ,
        EYE_CONTACT,
        JOINT_ATTENTION
    };
    const Type TYPE;

    LookatBehavior(Type type)
        : TYPE(type), BaseBehavior(selectTag(type), selectAutoOccuring(type)),
          FORCE_CHANGE_MS(selectForceChangeMs(type)) {}

    // Type selection (tag)
    std::string selectTag(Type type) {
        switch (type) {
            case Type::RAND_FACE: return "Lookat[RandomFace]";
            case Type::RAND_OBJ: return "Lookat[RandomObj]";
            case Type::CLOSEST_FACE: return "Lookat[ClosestFace]";
            case Type::CLOSEST_OBJ: return "Lookat[ClosestObj]";
            case Type::MOVING_FACE: return "Lookat[MovingFace]";
            case Type::MOVING_OBJ: return "Lookat[MovingObj]";
            case Type::EYE_CONTACT: return "Lookat[EyeContact]";
            case Type::JOINT_ATTENTION: return "Lookat[JointAttention]";
        }
        return "Lookat[?]";
    }

    // Type selection (auto_occurring)
    bool selectAutoOccuring(Type type) {
        switch (type) {
            case Type::RAND_FACE: return false;
            case Type::RAND_OBJ: return false;
            case Type::CLOSEST_FACE: return false;
            case Type::CLOSEST_OBJ: return false;
            case Type::MOVING_FACE: return false;
            case Type::MOVING_OBJ: return false;
            case Type::EYE_CONTACT: return false;
            case Type::JOINT_ATTENTION: return false;
        }
        return false;
    }

    // Type selection (FORCE_CHANGE_MS)
    float selectForceChangeMs(Type type) {
        switch (type) {
            case Type::RAND_FACE: return 3000.f;
            case Type::RAND_OBJ: return 3000.f;
            case Type::CLOSEST_FACE: return 500.f;
            case Type::CLOSEST_OBJ: return 500.f;
            case Type::MOVING_FACE: return 800.f;
            case Type::MOVING_OBJ: return 800.f;
            case Type::EYE_CONTACT: return 1000.f;
            case Type::JOINT_ATTENTION: return 1000.f;
        }
        return false;
    }

    // Static parameters
    const float FORCE_CHANGE_MS;
    const float SPEED_EYES = 7.0f, SPEED_HEAD = 1.8f, SPEED_BODY = 0.1f;
    const float ACCEL_EYES = 1.0f, ACCEL_HEAD = 0.3f, ACCEL_BODY = 0.01f;
    const float SPEED_EYELID = 3.f, ACCEL_EYELID = 0.3;  // eyelid
    const float MAX_CLOSEST_DISTANCE = 100.f;            // for closest types
    const float MIN_MOVING_VELOCITY = 1.0f;              // for moving types
    const float WAIT_MS = 50.f;

private:
    void declareWeights();
    bool shouldOccur();
    void behave();

    bool getNextLookatIdx(const SensorCtr& s, int& res_face_idx,
                          int& res_obj_idx);

    // common variables
    int last_face_idx = -1, last_obj_idx = -1;

    // for moving types
    std::vector<char> prev_face_valids, prev_object_valids;
    std::vector<glm::vec3> prev_face_poses, prev_object_poses;
};

// ===== Look Back Attention =====
class LookBackAttentionBehavior : public BaseBehavior {
public:
    LookBackAttentionBehavior() : BaseBehavior("LookBackAttention", false) {}

    // Look face (LOOKING_FACE_MS)
    // -> att_idx == NONE
    // -> look back (LOOKING_BACK_MS)

    // Static parameters
    const float SPEED_EYES = 7.0f, SPEED_HEAD = 1.8f, SPEED_BODY = 0.1f;
    const float ACCEL_EYES = 1.0f, ACCEL_HEAD = 0.3f, ACCEL_BODY = 0.01f;
    const float SPEED_EYELID = 3.f, ACCEL_EYELID = 0.3;  // eyelid
    const float WAIT_MS = 50.f;
    const float LOOKING_FACE_MS = 2000.f, LOOKING_BACK_MS = -1.f;
    const float HEAD_PAR = 0.2f;  // [0:1]

private:
    void declareWeights();
    bool shouldOccur();
    void behave();

    // regarding face info
    std::chrono::system_clock::time_point start_clock;
    int looking_face_idx;
};

// ===== Look at Body Behavior =====
class LookatBodyBehavior : public BaseBehavior {
public:
    LookatBodyBehavior() : BaseBehavior("LookatBody", false) {}

    // Static parameters
    const float SPEED = 0.6f, ACCEL = 0.1f;
    const float DEG = 40.f;
    const float WAIT_MS = 50.f;
    const float LOOKING_FACE_MS = 5000.f, LOOKING_BODY_MS = 3000.f;

private:
    void declareWeights();
    bool shouldOccur();
    void behave();

    // regarding face info
    std::chrono::system_clock::time_point start_clock;
    int looking_face_idx;
};

// ===== HelloVoice =====
class HelloVoiceBehavior : public BaseBehavior {
public:
    HelloVoiceBehavior() : BaseBehavior("HelloVoice", false) {}

    // Static parameters
    const float WAIT_MS = 2000.f;
    const int PROB_DENOM = 500;
    const std::string TEXT = "こんにちは";

private:
    void declareWeights();
    bool shouldOccur();
    void behave();
};

// ===== Breath =====
class BreathBehavior : public BaseBehavior {
public:
    BreathBehavior() : BaseBehavior("Breath", true) {}

    // Static parameters
    const float SPEED_HEAD = 0.01f, ACCEL_HEAD = 0.001f;
    const float SPEED_ARMS = 1.0f, ACCEL_ARMS = 0.01f;
    const float WAIT_MS = 2500.f;
    const float HEAD_DEG = 10.f, ARM_DEG = 10.f;

private:
    void declareWeights();
    bool shouldOccur();
    void behave();
};

// ===== Neck hold with saccades =====
class NeckHoldWithSaccades : public BaseBehavior {
public:
    NeckHoldWithSaccades() : BaseBehavior("NeckHoldWithSaccades", true) {}

    // Static parameters
    const float SPEED = 0.001f, ACCEL = 0.0001f;
    const float EMPTY_WAIT_MS = 20.f, HOLD_WAIT_MS = 100.f;
    const float THRESHOLD_DEG = 10.f;

private:
    void declareWeights();
    bool shouldOccur();
    void behave();
};

// ===== Personal space behavior =====
class PersonalSpaceBehavior : public BaseBehavior {
public:
    PersonalSpaceBehavior() : BaseBehavior("PersonalSpace", true) {}

    // Static parameters
    const float SPEED = 1.0f, ACCEL = 0.4f, WAIT_MS = 20.f;
    const float HEAD_DEG = 50.f;
    const float PERSONAL_SPACE = 30.f;

private:
    void declareWeights();
    bool shouldOccur();
    void behave();

    bool isSomethingInSpace(const SensorCtr &s);
};

#endif
