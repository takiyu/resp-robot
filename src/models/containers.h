#ifndef CONTAINERS_H_160920
#define CONTAINERS_H_160920

#include <boost/thread.hpp>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>

#include "../modules/modules.h"

// ======================== Thread Safe Copiable Pointer =======================
// Thread safe wrapper for deep copying.
template <typename T>
class ThreadCopiablePtr {
public:
    ThreadCopiablePtr() { t_ptr = std::make_shared<T>(); }
    // Thread safe methods and operators
    ThreadCopiablePtr(T* rhs) { t_ptr.reset(rhs); }
    ThreadCopiablePtr(const ThreadCopiablePtr& rhs) { *this = rhs; }
    ThreadCopiablePtr& operator=(const ThreadCopiablePtr& rhs) {
        std::unique_lock<std::mutex> l1(m, std::defer_lock);
        std::unique_lock<std::mutex> l2(rhs.m, std::defer_lock);
        std::lock(l1, l2);
        *t_ptr = *(rhs.t_ptr);
        return *this;
    }
    void set(const T& t) {
        std::unique_lock<std::mutex> l(m);
        *t_ptr = t;
    }
    void reset() { set(T()); }
    T copy() {
        std::unique_lock<std::mutex> l(m);
        return *t_ptr;
    }

    // Outer lock
    void lock() { m.lock(); }
    void unlock() { m.unlock(); }

    // Direct access operators (no lock)
    T& operator*() { return *t_ptr; }
    T* operator->() { return t_ptr.get(); }  // get pointer
    std::shared_ptr<T> t_ptr;

private:
    mutable std::mutex m;  // Avoid coping conflicts
};

// ============================== Sensor Container =============================
struct SensorCtr {
    // ====== Overwritable variables ======
    // Camera Scene
    std::vector<char> face_valids, object_valids;
    std::vector<glm::vec3> face_poses, object_poses;
    std::vector<CameraScene::AttentionIdx> att_idxs, last_att_idxs,
        last_obj_att_idxs;
    std::vector<glm::vec3> att_back_poses;
    // VAD
    char voice_active, voice_started, voice_finished;
    float voice_len_ms;
    // Voice Recognition
    char voice_recog_processing, voice_recoged;
    std::string voice_recog_result;

    // ====== Non-Overwritable variables ======
    // Robovie
    glm::mat4 robovie_body_tf_mat, robovie_head_tf_mat;
    glm::mat4 robovie_arm_l01_base_tf_mat, robovie_arm_r01_base_tf_mat;
    glm::vec3 robovie_head_pos, robovie_eyes_pos;
    float robovie_eyelid_per;
    glm::vec3 robovie_body_regard_pos, robovie_head_regard_pos,
        robovie_eyes_regard_pos;
    std::pair<glm::vec3, glm::vec3> robovie_left_arm_regard_poses;
    std::pair<glm::vec3, glm::vec3> robovie_right_arm_regard_poses;
};

// ============================ Movement Container =============================
const int N_BODY_PARTS = 7;  // depending on member variables in the container

struct MovementCtr {
    MovementCtr() : tag() {}

    // Movement part structure
    template <typename DataType>
    struct BasePart {
        BasePart()
            : empty(true), used(false), w(0.f), max_speed(-1.f), accel(-1.f) {}
        void set(const DataType& v, float w) {
            this->empty = false;
            this->used = false;
            this->v = v;
            this->w = w;
        }
        void setSpeed(float max_speed, float accel) {
            this->max_speed = max_speed;
            this->accel = accel;
        }

        bool empty, used;
        float w;
        DataType v;
        float max_speed, accel;

        // Merged history <tag, percentage>
        std::map<std::string, float> merged_history;
    };
    typedef BasePart<float> PercentPart;
    typedef BasePart<glm::vec3> RegardPosPart;
    typedef BasePart<std::string> StringPart;
    typedef BasePart<std::pair<glm::vec3, glm::vec3> > DoubleRegardPosPart;

    // Part variables
    RegardPosPart body;
    RegardPosPart head;
    RegardPosPart eyes;
    PercentPart eyelid;
    DoubleRegardPosPart right_arm;
    DoubleRegardPosPart left_arm;
    StringPart voice;

    // Get this container with used flag
    MovementCtr use() {
        MovementCtr b = *this;  // copy
        // set used flags
        body.used = head.used = eyes.used = eyelid.used = true;
        right_arm.used = left_arm.used = voice.used = true;
        return b;  // copy
    }

    bool isAllEmptyOrUsed() {
        return (body.empty || body.used) && (head.empty || head.used) &&
               (eyes.empty || eyes.used) && (eyelid.empty || eyelid.used) &&
               (right_arm.empty || right_arm.used) &&
               (left_arm.empty || left_arm.used) && (voice.empty || voice.used);
    }

    // Tag
    std::string tag;
};

// ======================= Movement Evaluation Container =======================
struct MovementEvaluationCtr {
    MovementEvaluationCtr() : tag() {}

    // Movement part structure
    template <typename DataType>
    struct BasePart {
        BasePart() { v.first = false; }
        // Value and/or behavior percentages
        std::pair<bool, DataType> v;                 // [use, value]
        std::map<std::string, float> behavior_pers;  // [tag, percentage]
    };
    typedef BasePart<float> PercentPart;
    typedef BasePart<glm::vec3> RegardPosPart;
    typedef BasePart<std::string> StringPart;
    typedef BasePart<std::pair<glm::vec3, glm::vec3> > DoubleRegardPosPart;

    // Part variables
    RegardPosPart body;
    RegardPosPart head;
    RegardPosPart eyes;
    PercentPart eyelid;
    DoubleRegardPosPart right_arm;
    DoubleRegardPosPart left_arm;
    StringPart voice;

    // Tag
    std::string tag;
};

#endif
