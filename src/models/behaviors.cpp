#include "behaviors.h"

#include <limits>

// TODO Move previous movement to the sensor value

namespace {
// Sleep
inline void sleepMillisec(float ms) {
    boost::this_thread::sleep(boost::posix_time::milliseconds(ms));
}

// Duration
inline float getPastMs(const std::chrono::system_clock::time_point& clk) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now() - clk)
        .count();
}

// Random (bool)
inline bool randBool(int prob) { return rand() % prob == 0; }

// Random (float)
inline float randFloat(float lower = -1.f, float upper = 1.f) {
    float diff = upper - lower;
    float rand01 = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    return diff * rand01 + lower;
}

// 2-dim length
inline float length(float x, float y) { return sqrtf(x * x + y * y); }

// Matrix transformation (direction)
inline glm::vec3 transformDir(const glm::vec3& dir, const glm::mat4& tf_mat) {
    return glm::vec3(tf_mat * glm::vec4(dir, 0));
}

// Matrix transformation (position)
inline glm::vec3 transformPos(const glm::vec3& dir, const glm::mat4& tf_mat) {
    return glm::vec3(tf_mat * glm::vec4(dir, 1));
}

// Rotate position with some axis
inline glm::vec3 rotatePos(const glm::vec3& src_pos, float deg,
                           const glm::vec3& axis,
                           const glm::mat4& center_tf_mat) {
    // Normalize -> Rotate -> Denormalize
    glm::vec3 pos = transformPos(src_pos, glm::inverse(center_tf_mat));
    pos = transformPos(pos, glm::rotate(glm::radians(deg), axis));
    return transformPos(pos, center_tf_mat);
}

// transformDir() for a pair
inline std::pair<glm::vec3, glm::vec3> transformDir(
    const std::pair<glm::vec3, glm::vec3>& dir, const glm::mat4& tf_mat) {
    return std::pair<glm::vec3, glm::vec3>(transformDir(dir.first, tf_mat),
                                           transformDir(dir.second, tf_mat));
}

// transformPos() for a pair
inline std::pair<glm::vec3, glm::vec3> transformPos(
    const std::pair<glm::vec3, glm::vec3>& pos, const glm::mat4& tf_mat) {
    return std::pair<glm::vec3, glm::vec3>(transformPos(pos.first, tf_mat),
                                           transformPos(pos.second, tf_mat));
}

// rotatePos() for a pair
inline std::pair<glm::vec3, glm::vec3> rotatePos(
    const std::pair<glm::vec3, glm::vec3>& pos, float deg,
    const glm::vec3& axis, const glm::mat4& center_tf_mat) {
    return std::pair<glm::vec3, glm::vec3>(
        rotatePos(pos.first, deg, axis, center_tf_mat),
        rotatePos(pos.second, deg, axis, center_tf_mat));
}

// degree in positions
inline float getDegreeInPositions(const glm::vec3& pos0, const glm::vec3& pos1,
                                 const glm::vec3& center_pos) {
    // Eyes target position with normalization
    glm::vec3 rel_pos0 = glm::normalize(pos0 - center_pos);
    glm::vec3 rel_pos1 = glm::normalize(pos1 - center_pos);

    // Calculate degree
    float distance = glm::distance(rel_pos0, rel_pos1);
    float deg = glm::degrees(fabsf(asinf(distance * 0.5f) * 2.f));
    return deg;
}

// check for valid indices
inline bool isValidIdx(int idx, const std::vector<char>& valids) {
    return 0 <= idx && idx < valids.size() && valids[idx];
}

// Valid indices extraction
inline void extractValidIdxs(const std::vector<char>& valids,
                             std::vector<int>& extracted_idxs) {
    for (int i = 0; i < valids.size(); i++) {
        if (valids[i]) extracted_idxs.push_back(i);
    }
}

inline int extractLookingFaceIdx(const glm::vec3& regard_pos,
                                 const std::vector<char>& face_valids,
                                 const std::vector<glm::vec3>& face_poses,
                                 const float threshold_cm) {
    float min_distance = std::numeric_limits<float>::max();
    int min_dist_idx = -1;
    for (int i = 0; i < face_valids.size(); i++) {
        if (!face_valids[i]) continue;
        float distance = glm::distance(regard_pos, face_poses[i]);
        if (distance < min_distance) {
            min_distance = distance;
            min_dist_idx = i;
        }
    }
    if (min_dist_idx != -1 && min_distance <= threshold_cm) {
        return min_dist_idx;
    } else {
        return -1;
    }
}

inline int findClosestIdx(const std::vector<char>& valids,
                          const std::vector<glm::vec3>& poses,
                          const glm::vec3& org_pos = glm::vec3(0.f),
                          float max_limit = std::numeric_limits<float>::max()) {
    float min_dist = max_limit;
    int min_dist_idx = -1;
    for (int i = 0; i < valids.size(); i++) {
        if (!valids[i]) continue;
        float dist = glm::length(poses[i] - org_pos);
        if (dist < min_dist) {
            min_dist = dist;
            min_dist_idx = i;
        }
    }
    return min_dist_idx;
}

inline int findMaxVelocityIdx(const std::vector<char>& valids,
                              const std::vector<char>& prev_valids,
                              const std::vector<glm::vec3>& poses,
                              const std::vector<glm::vec3>& prev_poses,
                              float min_limit) {
    float max_v = min_limit;
    int max_v_idx = -1;
    for (int i = 0; i < valids.size(); i++) {
        if (!valids[i] || !prev_valids[i]) continue;
        float v = glm::distance(poses[i], prev_poses[i]);
        if (max_v < v) {
            max_v = v;
            max_v_idx = i;
        }
    }
    // Detect appearance
    if (max_v_idx == -1) {
        std::vector<int> appeared_idxs;
        for (int i = 0; i < valids.size(); i++) {
            if (valids[i] && !prev_valids[i]) {
                appeared_idxs.push_back(i);
            }
        }
        if (appeared_idxs.size() > 0) {
            // Choose randomly
            max_v_idx = appeared_idxs[rand() % appeared_idxs.size()];
        }
    }
    return max_v_idx;
}
}

// ===== Sleep =====
void SleepBehavior::declareWeights() {
    setWeight("body", 0.1f, true);
    setWeight("head", 0.1f, true);
    setWeight("eyes", 0.1f, true);
    setWeight("eyelid", 0.1f, true);
    setWeight("rightarm", 0.1f, true);
    setWeight("leftarm", 0.1f, true);
    setWeight("voice", 0.1f, true);
}

bool SleepBehavior::shouldOccur() { return true; }

void SleepBehavior::behave() {
    // Coordinate parameters
    const std::pair<glm::vec3, glm::vec3> POS_RIGHT(glm::vec3(-11.f, 8.f, 3.f),
                                                    glm::vec3(-9.f, 3.f, 5.f));
    const std::pair<glm::vec3, glm::vec3> POS_LEFT(glm::vec3(11.f, 8.f, 3.f),
                                                   glm::vec3(9.f, 3.f, 5.f));

    while (true) {
        // Copy to local
        SensorCtr s = this->sensor_ctr.copy();

        MovementCtr b;
        // Move
        b.body.set(glm::vec3(0.f, 0.f, 100.f), getWeight("body"));
        //b.head.set(glm::vec3(0.f, 0.f, 100.f), getWeight("head"));
        //b.eyes.set(glm::vec3(0.f, 0.f, 100.f), getWeight("eyes"));
        b.eyelid.set(0.9f, getWeight("eyelid"));
        {
            // Consider body swing
            auto pos_right = transformPos(POS_RIGHT, s.robovie_body_tf_mat);
            auto pos_left = transformPos(POS_LEFT, s.robovie_body_tf_mat);
            // Set
            b.right_arm.set(pos_right, getWeight("rightarm"));
            b.left_arm.set(pos_left, getWeight("leftarm"));
        }
        b.voice.set("", getWeight("voice"));
        // Speed
        b.body.setSpeed(overwriteSpeed(SPEED), overwriteAccel(ACCEL));
        //b.head.setSpeed(overwriteSpeed(SPEED), overwriteAccel(ACCEL));
        //b.eyes.setSpeed(overwriteSpeed(SPEED), overwriteAccel(ACCEL));
        b.eyelid.setSpeed(overwriteSpeed(SPEED), overwriteAccel(ACCEL));
        b.right_arm.setSpeed(overwriteSpeed(SPEED), overwriteAccel(ACCEL));
        b.left_arm.setSpeed(overwriteSpeed(SPEED), overwriteAccel(ACCEL));
        this->behavior_ctr.set(b);

        // minimal interval
        sleepMillisec(overwriteMs(WAIT_MS));
    }
}

// ===== Move head =====
void MoveHeadBehavior::declareWeights() {
    setWeight("head", 1000.0f, true);  // Maximum weight
}

bool MoveHeadBehavior::shouldOccur() {
    if (auto_occurring) {
        return false;
    } else {
        return true;  // This behavior dose not occur automatically
    }
}

void MoveHeadBehavior::behave() {
    for (int i = 0; i < 1; i++) {
//     while (true) {
        // Copy to local
        SensorCtr s = this->sensor_ctr.copy();

        // Rotate X with head
        const glm::vec3 cur_p = s.robovie_head_regard_pos;
        glm::vec3 dst_p;
        switch (TYPE) {
            case Type::UP:
                dst_p = rotatePos(cur_p, -DEG, glm::vec3(1, 0, 0),
                                  s.robovie_head_tf_mat);
                break;
            case Type::DOWN:
                dst_p = rotatePos(cur_p, DEG, glm::vec3(1, 0, 0),
                                  s.robovie_head_tf_mat);
                break;
            case Type::RIGHT:
                dst_p = rotatePos(cur_p, -DEG, glm::vec3(0, 1, 0),
                                  s.robovie_head_tf_mat);
                break;
            case Type::LEFT:
                dst_p = rotatePos(cur_p, DEG, glm::vec3(0, 1, 0),
                                  s.robovie_head_tf_mat);
                break;
            default:
                dst_p = cur_p;
        }
        // Move
        MovementCtr b;
        // Set absolute position and speed
        b.head.set(dst_p, getWeight("head"));
        b.head.setSpeed(overwriteSpeed(SPEED), overwriteAccel(ACCEL));
        // Set
        this->behavior_ctr.set(b);

        // Minimal wait
        sleepMillisec(overwriteMs(WAIT_MS));
    }
}

// ===== Blink =====
void BlinkBehavior::declareWeights() {
    setWeight("eyelid_down", 10.f, true);
    setWeight("eyelid_up", 10.f, true);
}

bool BlinkBehavior::shouldOccur() {
    if (TYPE == Type::RAND) {
        return randBool(PROB_DENOM);
    } else if (TYPE == Type::DYNAMIC) {
        SensorCtr s = this->sensor_ctr.copy();
        // Calculate degree
        float deg = getDegreeInPositions(prev_eyes_regard_pos,
                                         s.robovie_eyes_regard_pos,
                                         s.robovie_eyes_pos);
        // Update history
        prev_eyes_regard_pos = s.robovie_eyes_regard_pos;
        // Threshold
        return THRESHOLD_DEG <= deg;
    } else {
        return false;
    }
}

void BlinkBehavior::behave() {
    // Copy to local
    SensorCtr s = this->sensor_ctr.copy();

    // Calculate eyelid current and destination percentages
    float org_per = std::min(s.robovie_eyelid_per, 1.f);
    if (org_per <= 0.f) return;  // no need to blink
    float dst_per = org_per - PER;

    // down
    {
        MovementCtr b;
        b.eyelid.setSpeed(overwriteSpeed(SPEED), overwriteAccel(ACCEL));
        b.eyelid.set(dst_per, getWeight("eyelid_down"));
        this->behavior_ctr.set(b);
        sleepMillisec(overwriteMs(WAIT_MS));
    }

    // up
    {
        MovementCtr b;
        b.eyelid.setSpeed(overwriteSpeed(SPEED), overwriteAccel(ACCEL));
        b.eyelid.set(org_per, getWeight("eyelid_up"));
        this->behavior_ctr.set(b);
        sleepMillisec(overwriteMs(WAIT_MS));
    }
}

// ===== Nod =====
void NodBehavior::declareWeights() {
    setWeight("head_down", 2.f, true);
    setWeight("head_up", 0.5f, true);
}

bool NodBehavior::shouldOccur() {
    // Check face and voice
    SensorCtr s = this->sensor_ctr.copy();
    std::vector<int> valid_idxs;
    extractValidIdxs(s.face_valids, valid_idxs);
    return (valid_idxs.size() > 0) && (s.voice_finished) &&
           (s.voice_len_ms > 500.f);
}

void NodBehavior::behave() {
    // TODO Use some face

    // Down and up
    for (int i = 0; i < 2; i++) {
        // Copy to local
        SensorCtr s = this->sensor_ctr.copy();
        // Move
        MovementCtr b;
        // Rotate X with head
        glm::vec3 p =
            rotatePos(s.robovie_head_regard_pos, DEG * (i == 0 ? 1.f : -1.f),
                      glm::vec3(1, 0, 0), s.robovie_head_tf_mat);
        // Set absolute position and speed
        b.head.set(p, (i == 0 ? getWeight("head_down") : getWeight("head_up")));
        b.head.setSpeed(overwriteSpeed(SPEED), overwriteAccel(ACCEL));
        // Set
        this->behavior_ctr.set(b);
        sleepMillisec(overwriteMs(WAIT_MS));
    }
}

// ===== HandWave =====
void HandWaveBehavior::declareWeights() {
    if (TYPE == Type::BOTH_RAND) {
        float w;
        if (auto_occurring) {
            w = 1.6f;
        } else {
            w = 1001.0f;  // Maximum weight
        }
        setWeight("inner_l", w, true);
        setWeight("outer_l", w, true);
        setWeight("inner_r", w, true);
        setWeight("outer_r", w, true);
    } else if (TYPE == Type::LEFT_RAND || TYPE == Type::RIGHT_RAND) {
        float w;
        if (auto_occurring) {
            w = 1.5f;
        } else {
            w = 501.0f;  // Semi-maximum weight
        }
        setWeight("inner", w, true);
        setWeight("outer", w, true);
    }
    else if (TYPE == Type::BOTH_RESP) {
        setWeight("inner_l", 2.5f, true);
        setWeight("outer_l", 2.5f, true);
        setWeight("inner_r", 2.5f, true);
        setWeight("outer_r", 2.5f, true);
    } else if (TYPE == Type::LEFT_RESP || TYPE == Type::RIGHT_RESP) {
        setWeight("inner", 0.3f, true);
        setWeight("outer", 0.3f, true);
    }
}

bool HandWaveBehavior::shouldOccur() {
    if (TYPE == Type::BOTH_RAND || TYPE == Type::LEFT_RAND ||
        TYPE == Type::RIGHT_RAND) {
        if (auto_occurring) {
            // Check there is some valid face
            SensorCtr s = this->sensor_ctr.copy();
            std::vector<int> valid_idxs;
            extractValidIdxs(s.face_valids, valid_idxs);
            return (valid_idxs.size() > 0) && randBool(PROB_DENOM);
        } else {
            // No check to occur because of manual manipulation
            return true;
        }
    }
    else if (TYPE == Type::BOTH_RESP || TYPE == Type::LEFT_RESP ||
        TYPE == Type::RIGHT_RESP) {
        // Check there is some valid face and hand
        SensorCtr s = this->sensor_ctr.copy();
        std::vector<int> valid_face_idxs;
        extractValidIdxs(s.face_valids, valid_face_idxs);
        return (valid_face_idxs.size() > 0 && 
                s.object_valids.size() >= 3 && s.object_valids[2]);
    }
    else {
        printf(" >> Invalid type (HandWaveBehavior)\n");
        return false;
    }
}

void HandWaveBehavior::behave() {
    // Coordinate parameters
    const std::pair<glm::vec3, glm::vec3> INNER_POS_LEFT(
        glm::vec3(12.f, 20.f, 7.f), glm::vec3(9.f, 25.f, 11.f));
    const std::pair<glm::vec3, glm::vec3> INNER_POS_RIGHT(  // x flip of left
        glm::vec3(-12.f, 20.f, 7.f), glm::vec3(-9.f, 25.f, 11.f));
    const std::pair<glm::vec3, glm::vec3> OUTER_POS_LEFT(
        glm::vec3(12.f, 20.f, 7.f), glm::vec3(15.f, 25.f, 11.f));
    const std::pair<glm::vec3, glm::vec3> OUTER_POS_RIGHT(  // x flip of left
        glm::vec3(-12.f, 20.f, 7.f), glm::vec3(-15.f, 25.f, 11.f));

    int n = (auto_occurring ? N_WAVE_AUTO : N_WAVE);
    for (int i = 0; i < n; i++) {
        std::cout << i << std::endl;
        // Inner
        {
            // Copy to local
            SensorCtr s = this->sensor_ctr.copy();
            MovementCtr b;
            // Left
            if (TYPE == Type::LEFT_RAND || TYPE == Type::BOTH_RAND ||
                TYPE == Type::LEFT_RESP || TYPE == Type::BOTH_RESP) {
                // Consider body swing
                auto pos = transformPos(INNER_POS_LEFT, s.robovie_body_tf_mat);
                // Weight label
                std::string w_str =
                    (TYPE == Type::BOTH_RAND || TYPE == Type::BOTH_RESP) ?
                    "inner_l" : "inner";
                // Set
                b.left_arm.setSpeed(overwriteSpeed(SPEED),
                                    overwriteAccel(ACCEL));
                b.left_arm.set(pos, getWeight(w_str));
            }
            // Right
            if (TYPE == Type::RIGHT_RAND || TYPE == Type::BOTH_RAND ||
                TYPE == Type::RIGHT_RESP || TYPE == Type::BOTH_RESP) {
                // Consider body swing
                auto pos = transformPos(INNER_POS_RIGHT, s.robovie_body_tf_mat);
                // Weight label
                std::string w_str =
                    (TYPE == Type::BOTH_RAND || TYPE == Type::BOTH_RESP) ?
                    "inner_r" : "inner";
                // Set
                b.right_arm.setSpeed(overwriteSpeed(SPEED),
                                     overwriteAccel(ACCEL));
                b.right_arm.set(pos, getWeight(w_str));
            }
            this->behavior_ctr.set(b);
            sleepMillisec(overwriteMs(WAIT_MS));
        }
        // Outer
        {
            // Copy to local
            SensorCtr s = this->sensor_ctr.copy();
            MovementCtr b;
            // Left
            if (TYPE == Type::LEFT_RAND || TYPE == Type::BOTH_RAND ||
                TYPE == Type::LEFT_RESP || TYPE == Type::BOTH_RESP) {
                // Consider body swing
                auto pos = transformPos(OUTER_POS_LEFT, s.robovie_body_tf_mat);
                // Weight label
                std::string w_str =
                    (TYPE == Type::BOTH_RAND || TYPE == Type::BOTH_RESP) ?
                    "outer_l" : "outer";
                // Set
                b.left_arm.setSpeed(overwriteSpeed(SPEED),
                                    overwriteAccel(ACCEL));
                b.left_arm.set(pos, getWeight(w_str));
            }
            // Right
            if (TYPE == Type::RIGHT_RAND || TYPE == Type::BOTH_RAND ||
                TYPE == Type::RIGHT_RESP || TYPE == Type::BOTH_RESP) {
                // Consider body swing
                auto pos = transformPos(OUTER_POS_RIGHT, s.robovie_body_tf_mat);
                // Weight label
                std::string w_str =
                    (TYPE == Type::BOTH_RAND || TYPE == Type::BOTH_RESP) ?
                    "outer_r" : "outer";
                // Set
                b.right_arm.setSpeed(overwriteSpeed(SPEED),
                                     overwriteAccel(ACCEL));
                b.right_arm.set(pos, getWeight(w_str));
            }
            this->behavior_ctr.set(b);
            sleepMillisec(overwriteMs(WAIT_MS));
        }
    }
}

// ===== Hand Holding out =====
void HandHoldOutBehavior::declareWeights() {
    setWeight("right", 500.0f, true);  // Semi-maximum weight
    setWeight("left", 500.0f, true);  // Semi-maximum weight
    setWeight("both", 1000.0f, true);  // Maximum weight
}

bool HandHoldOutBehavior::shouldOccur() {
    if (auto_occurring) {
        return false;
    } else {
        return true;  // This behavior dose not occur automatically
    }
}

void HandHoldOutBehavior::behave() {
    const std::pair<glm::vec3, glm::vec3> POS_RIGHT(glm::vec3(-11.f, 20.f, 5.f),
                                                    glm::vec3(-11.f, 20.f, 20.f));
    const std::pair<glm::vec3, glm::vec3> POS_LEFT(glm::vec3(11.f, 20.f, 5.f),
                                                   glm::vec3(11.f, 20.f, 20.f));

    while (true) {
        // Copy to local
        SensorCtr s = this->sensor_ctr.copy();

        // Move
        MovementCtr b;

#if 0
        // Consider body swing
        auto pos_right = transformPos(POS_RIGHT, s.robovie_body_tf_mat);
        auto pos_left = transformPos(POS_LEFT, s.robovie_body_tf_mat);
#else
        // No considering
        auto pos_right = POS_RIGHT;
        auto pos_left = POS_LEFT;
#endif

        // Set position and speed
        switch (TYPE) {
            case Type::RIGHT:
                b.right_arm.set(pos_right, getWeight("right"));
                b.right_arm.setSpeed(overwriteSpeed(SPEED),
                                     overwriteAccel(ACCEL));
                break;
            case Type::LEFT:
                b.left_arm.set(pos_left, getWeight("left"));
                b.left_arm.setSpeed(overwriteSpeed(SPEED),
                                    overwriteAccel(ACCEL));
                break;
            case Type::BOTH:
                b.right_arm.set(pos_right, getWeight("right"));
                b.right_arm.setSpeed(overwriteSpeed(SPEED),
                                     overwriteAccel(ACCEL));
                b.left_arm.set(pos_left, getWeight("left"));
                b.left_arm.setSpeed(overwriteSpeed(SPEED),
                                    overwriteAccel(ACCEL));
                break;
            default:
                break;
        }

        // Set
        this->behavior_ctr.set(b);

        // Minimal wait
        sleepMillisec(overwriteMs(WAIT_MS));
    }
}

// ===== Look at =====
void LookatBehavior::declareWeights() {
    float w_eyes, w_head, w_body;
    switch (TYPE) {
        // Face tracking
        case Type::RAND_FACE:
        case Type::RAND_OBJ:
        case Type::CLOSEST_FACE:
        case Type::CLOSEST_OBJ:
            w_eyes = 1.0f;
            w_head = 0.7f;
            w_body = 0.3f;
            break;
        case Type::MOVING_FACE:
        case Type::MOVING_OBJ:
            w_eyes = 1.1f;
            w_head = 0.8f;
            w_body = 0.4f;
            break;
        case Type::EYE_CONTACT:
            w_eyes = 1.2f;
            w_head = 0.9f;
            w_body = 0.5f;
            break;
        case Type::JOINT_ATTENTION:
            w_eyes = 1.3f;
            w_head = 1.0f;
            w_body = 0.6f;
            break;
    }
    setWeight("eyes", w_eyes, true);
    setWeight("head", w_head, true);
    setWeight("body", w_body, true);
    setWeight("eyelid", 0.9f, true);
}

bool LookatBehavior::shouldOccur() {
    SensorCtr s = this->sensor_ctr.copy();
    // Find lookat index
    last_face_idx = -1;
    last_obj_idx = -1;
    bool ret = getNextLookatIdx(s, last_face_idx, last_obj_idx);
    // If something is found with the type, should occur.
    return ret;
}

void LookatBehavior::behave() {
    // History for a next loop
    auto changed_clk = std::chrono::system_clock::now();

    while (true) {
        // ===== Force change of lookat target =====
        if (last_face_idx != -1) {
            if (overwriteMs(FORCE_CHANGE_MS) < getPastMs(changed_clk)) {
                changed_clk = std::chrono::system_clock::now();
                last_face_idx = -1;
            }
        }
        if (last_obj_idx != -1) {
            if (overwriteMs(FORCE_CHANGE_MS) < getPastMs(changed_clk)) {
                changed_clk = std::chrono::system_clock::now();
                last_obj_idx = -1;
            }
        }

        // ===== Current Status =====
        SensorCtr s = this->sensor_ctr.copy();
        bool do_lookat = false;
        glm::vec3 lookat_pos;
        // Tracking status
        bool last_face_valid = isValidIdx(last_face_idx, s.face_valids);
        bool last_obj_valid = isValidIdx(last_obj_idx, s.object_valids);
        // Clear the history
        if (!last_face_valid) last_face_idx = -1;
        if (!last_obj_valid) last_obj_idx = -1;

        // ===== Lookat patterns =====
        // === Track ===
        switch (TYPE) {
            // Face tracking
            case Type::RAND_FACE:
            case Type::CLOSEST_FACE:
            case Type::MOVING_FACE:
            case Type::EYE_CONTACT:
                if (last_face_valid) {
                    do_lookat = true;
                    lookat_pos = s.face_poses[last_face_idx];
                }
                break;
            // Object tracking
            case Type::RAND_OBJ:
            case Type::CLOSEST_OBJ:
            case Type::MOVING_OBJ:
            case Type::JOINT_ATTENTION:
                if (last_obj_valid) {
                    do_lookat = true;
                    lookat_pos = s.object_poses[last_obj_idx];
                }
                break;
        }
        // === New face or object ===
        if (!do_lookat) {
            // Search indices
            int next_face_idx = -1, next_obj_idx = -1;
            bool ret = getNextLookatIdx(s, next_face_idx, next_obj_idx);
            // Set
            if (ret) {
                if (next_face_idx != -1) {
                    last_face_idx = next_face_idx;
                    do_lookat = true;
                    lookat_pos = s.face_poses[last_face_idx];
                } else if (next_obj_idx != -1) {
                    last_obj_idx = next_obj_idx;
                    do_lookat = true;
                    lookat_pos = s.object_poses[last_obj_idx];
                }
            } else {
                // Exit looking
                return;
            }
        }

        // ===== Move =====
        if (do_lookat) {
            MovementCtr b;
            b.eyes.set(lookat_pos, getWeight("eyes"));
            b.head.set(lookat_pos, getWeight("head"));
            b.body.set(lookat_pos, getWeight("body"));
            b.eyes.setSpeed(overwriteSpeed(SPEED_EYES),
                            overwriteAccel(ACCEL_EYES));
            b.head.setSpeed(overwriteSpeed(SPEED_HEAD),
                            overwriteAccel(ACCEL_HEAD));
            b.body.setSpeed(overwriteSpeed(SPEED_BODY),
                            overwriteAccel(ACCEL_BODY));

            // open the eyes
            b.eyelid.set(0.9f, getWeight("eyelid"));
            b.eyelid.setSpeed(overwriteSpeed(SPEED_EYELID),
                              overwriteAccel(ACCEL_EYELID));

            this->behavior_ctr.set(b);
        }

        sleepMillisec(overwriteMs(WAIT_MS));
    }
}

bool LookatBehavior::getNextLookatIdx(const SensorCtr& s, int& res_face_idx,
                                      int& res_obj_idx) {
    // Switch with the type
    if (TYPE == Type::RAND_FACE) {
        // Random valid face
        std::vector<int> valid_idxs;
        extractValidIdxs(s.face_valids, valid_idxs);
        if (valid_idxs.size() > 0) {
            res_face_idx = valid_idxs[rand() % valid_idxs.size()];
            return true;
        }
    } else if (TYPE == Type::RAND_OBJ) {
        // Random valid object
        std::vector<int> valid_idxs;
        extractValidIdxs(s.object_valids, valid_idxs);
        if (valid_idxs.size() > 0) {
            res_obj_idx = valid_idxs[rand() % valid_idxs.size()];
            return true;
        }
    } else if (TYPE == Type::CLOSEST_FACE) {
        // Closest face
        int closest = findClosestIdx(s.face_valids, s.face_poses,
                                     s.robovie_head_pos, MAX_CLOSEST_DISTANCE);
        if (closest != -1) {
            res_face_idx = closest;
            return true;
        }
    } else if (TYPE == Type::CLOSEST_OBJ) {
        // Closest object
        int closest = findClosestIdx(s.object_valids, s.object_poses,
                                     s.robovie_head_pos, MAX_CLOSEST_DISTANCE);
        if (closest != -1) {
            res_obj_idx = closest;
            return true;
        }
    } else if (TYPE == Type::MOVING_FACE) {
        // Moving face
        // Allocate
        int n_faces = s.face_valids.size();
        prev_face_valids.resize(n_faces, false);
        prev_face_poses.resize(n_faces);
        // Search
        int max_v_idx = findMaxVelocityIdx(s.face_valids, prev_face_valids,
                                           s.face_poses, prev_face_poses,
                                           MIN_MOVING_VELOCITY);
        // Backup
        prev_face_valids = s.face_valids;
        prev_face_poses = s.face_poses;
        if (max_v_idx != -1) {
            res_face_idx = max_v_idx;
            return true;
        }
    } else if (TYPE == Type::MOVING_OBJ) {
        // Moving object
        // Allocate
        int n_objs = s.object_valids.size();
        prev_object_valids.resize(n_objs, false);
        prev_object_poses.resize(n_objs);
        // Search
        int max_v_idx = findMaxVelocityIdx(s.object_valids, prev_object_valids,
                                           s.object_poses, prev_object_poses,
                                           MIN_MOVING_VELOCITY);
        // Backup
        prev_object_valids = s.object_valids;
        prev_object_poses = s.object_poses;
        if (max_v_idx != -1) {
            res_obj_idx = max_v_idx;
            return true;
        }
    } else if (TYPE == Type::EYE_CONTACT) {
        // Eye contact (face)
        std::vector<int> contact_idxs;
        // Search
        for (int i = 0; i < s.face_valids.size(); i++) {
            if (!s.face_valids[i]) continue;
            if (s.att_idxs[i] == CameraScene::ROBOVIE) {
                contact_idxs.push_back(i);
            }
        }
        // Select one randomly
        if (contact_idxs.size() > 0) {
            res_face_idx = contact_idxs[rand() % contact_idxs.size()];
            return true;
        }
    } else if (TYPE == Type::JOINT_ATTENTION) {
        // Joint attention (object)
        std::vector<int> attended_idxs;
        // Search attending face
        for (int i = 0; i < s.face_valids.size(); i++) {
            if (!s.face_valids[i]) continue;
            if (s.att_idxs[i] >= CameraScene::OBJECT) {
                int obj_idx = s.att_idxs[i] - CameraScene::OBJECT;
                attended_idxs.push_back(obj_idx);
            }
        }
        // Select one randomly
        if (attended_idxs.size() > 0) {
            res_obj_idx = attended_idxs[rand() % attended_idxs.size()];
            return true;
        }
    } else {
        printf("* Invalid LookatBehavior type (%d)", static_cast<int>(TYPE));
    }
    return false;
}

// ===== LookBackAttentionBehavior =====
void LookBackAttentionBehavior::declareWeights() {
    setWeight("eyes", 2.f, true);
    setWeight("head", 2.f, true);
    setWeight("body", 2.f, true);
    setWeight("eyelid", 0.9f, true);
}

bool LookBackAttentionBehavior::shouldOccur() {
    SensorCtr s = this->sensor_ctr.copy();
    // Detect looking face
    int idx = extractLookingFaceIdx(s.robovie_eyes_regard_pos, s.face_valids,
                                    s.face_poses, 5.f);
    if (idx != -1) {
        start_clock = std::chrono::system_clock::now();
        looking_face_idx = idx;
        return true;
    } else {
        return false;
    }
}

void LookBackAttentionBehavior::behave() {
    glm::vec3 origin_regard_head_pos;

    if (auto_occurring) {
        // Continue looking at face
        while (true) {
            // Current looking face
            SensorCtr s = this->sensor_ctr.copy();
            int idx = extractLookingFaceIdx(s.robovie_eyes_regard_pos,
                                            s.face_valids, s.face_poses, 5.f);
            if (idx != looking_face_idx) {
                return;
            }

            // Check elapsed milliseconds
            float elapsed_ms = getPastMs(start_clock);
            if (elapsed_ms > overwriteMs(LOOKING_FACE_MS)) {
                // Backup original regard pos
                origin_regard_head_pos = s.robovie_head_regard_pos;
                // Go to next step
                break;
            }

            // Wait
            sleepMillisec(overwriteMs(WAIT_MS));
        }
    } else {
        // Look back in any case
        SensorCtr s = this->sensor_ctr.copy();
        std::vector<int> valid_idxs;
        extractValidIdxs(s.face_valids, valid_idxs);
        if (valid_idxs.size() == 0) {
            return;
        }
        looking_face_idx = valid_idxs[0];
    }

    // Wait for looking backward
    while (true) {
        SensorCtr s = this->sensor_ctr.copy();
        if (s.att_idxs[looking_face_idx] == CameraScene::AttentionIdx::NONE) {
            break;  // start looking back
        }
        // Wait
        sleepMillisec(overwriteMs(WAIT_MS));
    }

    // Look back
    std::chrono::system_clock::time_point lookback_clock =
        std::chrono::system_clock::now();
    while (true) {
        // Update looking position
        SensorCtr s = this->sensor_ctr.copy();
        glm::vec3 &lookat_pos = s.att_back_poses[looking_face_idx];
        glm::vec3 lookat_pos_head =  // composed position
            lookat_pos * HEAD_PAR + origin_regard_head_pos * (1.f - HEAD_PAR);

        // Move
        MovementCtr b;
        {
            // Regard
            b.eyes.set(lookat_pos, getWeight("eyes"));
            b.head.set(lookat_pos_head, getWeight("head"));
            b.body.set(lookat_pos_head, getWeight("body"));
            b.eyes.setSpeed(overwriteSpeed(SPEED_EYES),
                            overwriteAccel(ACCEL_EYES));
            b.head.setSpeed(overwriteSpeed(SPEED_HEAD),
                            overwriteAccel(ACCEL_HEAD));
            b.body.setSpeed(overwriteSpeed(SPEED_BODY),
                            overwriteAccel(ACCEL_BODY));
            // Open the eyes
            b.eyelid.set(0.9f, getWeight("eyelid"));
            b.eyelid.setSpeed(overwriteSpeed(SPEED_EYELID),
                              overwriteAccel(ACCEL_EYELID));
            this->behavior_ctr.set(b);
        }

        // Check elapsed milliseconds
        float elapsed_ms = getPastMs(lookback_clock);
        if (elapsed_ms > overwriteMs(LOOKING_BACK_MS) && LOOKING_BACK_MS >= 0) {
            break;  // exit looking back
        }

        // Check other attention
        if (s.face_valids[looking_face_idx] &&
            s.att_idxs[looking_face_idx] != CameraScene::AttentionIdx::NONE) {
            break;  // start looking back
        }

        // Wait
        sleepMillisec(overwriteMs(WAIT_MS));
    }
}

// ===== Look Body Behavior =====
void LookatBodyBehavior::declareWeights() {
    setWeight("eyes_down", 2.0f, true);
    setWeight("eyes_up", 2.0f, true);
}

bool LookatBodyBehavior::shouldOccur() {
    SensorCtr s = this->sensor_ctr.copy();
    // Detect looking face
    int idx = extractLookingFaceIdx(s.robovie_eyes_regard_pos, s.face_valids,
                                    s.face_poses, 5.f);
    if (idx != -1) {
        start_clock = std::chrono::system_clock::now();
        looking_face_idx = idx;
        return true;
    } else {
        return false;
    }
}

void LookatBodyBehavior::behave() {

    // Continue looking at face
    while (true) {
        // Current looking face
        SensorCtr s = this->sensor_ctr.copy();
        int idx = extractLookingFaceIdx(s.robovie_eyes_regard_pos,
                                        s.face_valids, s.face_poses, 5.f);
        if (idx != looking_face_idx) {
            return;
        }

        // Check elapsed milliseconds
        float elapsed_ms = getPastMs(start_clock);
        if (elapsed_ms > overwriteMs(LOOKING_FACE_MS)) {
            // Go to next step
            break;
        }

        // Wait
        sleepMillisec(overwriteMs(WAIT_MS));
    }

    // Look Down and up with eyes
    for (int i = 0; i < 2; i++) {
        // Copy to local
        SensorCtr s = this->sensor_ctr.copy();
        // Move
        MovementCtr b;
        // Rotate X with eyes
        glm::vec3 p =
            rotatePos(s.robovie_eyes_regard_pos, DEG * (i == 0 ? 1.f : -1.f),
                      glm::vec3(1, 0, 0), s.robovie_head_tf_mat);
        // Set absolute position and speed
        b.eyes.set(p, (i == 0 ? getWeight("eyes_down") : getWeight("eyes_up")));
        b.eyes.setSpeed(overwriteSpeed(SPEED), overwriteAccel(ACCEL));
        // Set
        this->behavior_ctr.set(b);
        sleepMillisec(overwriteMs(LOOKING_BODY_MS * 0.5f));
    }
}

// ===== HelloVoice =====
void HelloVoiceBehavior::declareWeights() { setWeight("voice", 1.0f, true); }

bool HelloVoiceBehavior::shouldOccur() {
    // Check there is some valid face
    SensorCtr s = this->sensor_ctr.copy();
    std::vector<int> valid_idxs;
    extractValidIdxs(s.face_valids, valid_idxs);
    return (valid_idxs.size() > 0) && randBool(PROB_DENOM);
}

void HelloVoiceBehavior::behave() {
    {
        MovementCtr b;
        b.voice.set(TEXT, getWeight("voice"));
        this->behavior_ctr.set(b);
        sleepMillisec(overwriteMs(WAIT_MS));
    }
}

// ===== Breath =====
void BreathBehavior::declareWeights() {
    setWeight("head", 0.10, true);
    setWeight("left_arm", 0.10, true);
    setWeight("right_arm", 0.10, true);
}

bool BreathBehavior::shouldOccur() { return true; }

void BreathBehavior::behave() {
    float dir = -1.f;
    while (true) {
        // Copy to local
        SensorCtr s = this->sensor_ctr.copy();

        // Move
        MovementCtr b;
        // Head
        {
            // Rotate X with head direction normalization
            glm::vec3 p = rotatePos(s.robovie_head_regard_pos, HEAD_DEG * dir,
                                    glm::vec3(1, 0, 0), s.robovie_head_tf_mat);
            // Set absolute position and speed
//             b.head.set(p, getWeight("head"));
//             b.head.setSpeed(overwriteSpeed(SPEED_HEAD),
//                             overwriteAccel(ACCEL_HEAD));
        }

        // Left arm
        {
            // Rotate Z
            auto poses =
                rotatePos(s.robovie_left_arm_regard_poses, ARM_DEG * dir * -1.f,
                          glm::vec3(0, 0, 1), s.robovie_arm_l01_base_tf_mat);
            // Set absolute position and speed
            b.left_arm.set(poses, getWeight("left_arm"));
            b.left_arm.setSpeed(overwriteSpeed(SPEED_ARMS),
                                overwriteAccel(ACCEL_ARMS));
        }

        // Right arm
        {
            // Rotate Z
            auto poses =
                rotatePos(s.robovie_right_arm_regard_poses, ARM_DEG * dir,
                          glm::vec3(0, 0, 1), s.robovie_arm_r01_base_tf_mat);
            // Set absolute position and speed
            b.right_arm.set(poses, getWeight("right_arm"));
            b.right_arm.setSpeed(overwriteSpeed(SPEED_ARMS),
                                 overwriteAccel(ACCEL_ARMS));
        }

        // Set and wait
        this->behavior_ctr.set(b);
        sleepMillisec(overwriteMs(WAIT_MS));

        // Change moving direction
        if (dir > 0.f) {
            dir = -1.f;
        } else {
            dir = 1.f;
        }
    }
}

// ===== Neck hold with saccades =====
void NeckHoldWithSaccades::declareWeights() {
    setWeight("head", 10.0, true);
}

bool NeckHoldWithSaccades::shouldOccur() { return true; }

void NeckHoldWithSaccades::behave() {
    // Regarding
    glm::vec3 prev_eyes_regard_pos, prev_head_regard_pos;
    {
        SensorCtr s = this->sensor_ctr.copy();
        prev_eyes_regard_pos = s.robovie_eyes_regard_pos;
        prev_head_regard_pos = s.robovie_head_regard_pos;
    }

    while (true) {
        // Copy to local
        SensorCtr s = this->sensor_ctr.copy();

        // Calculate degree
        float deg = getDegreeInPositions(prev_eyes_regard_pos,
                                         s.robovie_eyes_regard_pos,
                                         s.robovie_eyes_pos);

        // Head
        float sleep_ms;
        if (THRESHOLD_DEG <= deg) {
            // Set holding movement
            MovementCtr b;
            b.head.set(prev_head_regard_pos, getWeight("head"));
            b.head.setSpeed(overwriteSpeed(SPEED), overwriteAccel(ACCEL));
            this->behavior_ctr.set(b);
            sleep_ms = overwriteMs(HOLD_WAIT_MS);
        } else {
            // Set empty movement
            MovementCtr b;
            this->behavior_ctr.set(b);
            sleep_ms = overwriteMs(EMPTY_WAIT_MS);
        }

        // Update for next detection
        prev_eyes_regard_pos = s.robovie_eyes_regard_pos;
        prev_head_regard_pos = s.robovie_head_regard_pos;

        // Sleep
        sleepMillisec(sleep_ms);
    }
}

// ===== Personal space behavior =====
void PersonalSpaceBehavior::declareWeights() {
    setWeight("head", 20.00, true);
}

bool PersonalSpaceBehavior::shouldOccur() {
    SensorCtr s = this->sensor_ctr.copy();
    return isSomethingInSpace(s);
}

void PersonalSpaceBehavior::behave() {
    while (true) {
        // Copy to local
        SensorCtr s = this->sensor_ctr.copy();

        // Exit behavior
        bool ret = isSomethingInSpace(s);
        if (!ret) break;

        // Move
        MovementCtr b;
        // Head
        {
            // Rotate X (eyes position -> rotated head position)
            glm::vec3 p = rotatePos(s.robovie_eyes_regard_pos, HEAD_DEG * -1.f,
                                    glm::vec3(1, 0, 0), s.robovie_head_tf_mat);
            // Set absolute position and speed
            b.head.set(p, getWeight("head"));
            b.head.setSpeed(overwriteSpeed(SPEED), overwriteAccel(ACCEL));
        }

        // Set and wait
        this->behavior_ctr.set(b);
        sleepMillisec(overwriteMs(WAIT_MS));
    }
}

bool PersonalSpaceBehavior::isSomethingInSpace(const SensorCtr &s) {
    // Closest face in personal space
    int closest_face = findClosestIdx(s.face_valids, s.face_poses,
                                      s.robovie_head_pos, PERSONAL_SPACE);
    // Closest object in personal space
    int closest_obj = findClosestIdx(s.object_valids, s.object_poses,
                                     s.robovie_head_pos, PERSONAL_SPACE);

    // Check some faces or objects is in personal space
    if (closest_face != -1 || closest_obj != -1) {
        return true;
    } else {
        return false;
    }
}
