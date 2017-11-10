#include "mergers.h"

namespace {
// Cross operator for std::pair
template <typename T, typename S>
inline std::pair<T, T> operator*(const std::pair<T, T>& v, S s) {
    return std::pair<T, T>(v.first * s, v.second * s);
}

// Adding operator for std::pair
template <typename T>
inline std::pair<T, T> operator+(const std::pair<T, T>& v1,
                                 const std::pair<T, T>& v2) {
    return std::pair<T, T>(v1.first + v2.first, v1.second + v2.second);
}

// Priority
template <typename T>
void UpdateMaxPriority(float& max_priority, int& max_idx,
                       const MovementCtr::BasePart<T>& part, int idx) {
    if (!part.empty && max_priority < part.w) {
        max_idx = idx;
        max_priority = part.w;
    }
}

// Weight
template <typename T>
void UpdateSumWeight(float& sum_w, const MovementCtr::BasePart<T>& part) {
    if (!part.empty && 0.f < part.w) {
        sum_w += part.w;
    }
}

// Weight
template <typename T>
void AccumMovementPart(MovementCtr::BasePart<T>& dst,
                       const MovementCtr::BasePart<T>& src, float sum_weight) {
    if (!src.empty && sum_weight > 0.f) {
        float weight = src.w / sum_weight;
        // Accumulate with weight
        T v = src.v * weight;
        float w = src.w * weight;
        float max_speed = src.max_speed * weight;
        float accel = src.accel * weight;
        // Add Current values
        if (!dst.empty) {
            v = v + dst.v;
            w = w + dst.w;
            max_speed = max_speed + dst.max_speed;
            accel = accel + dst.accel;
        }
        // Set
        dst.set(v, w);
        dst.setSpeed(max_speed, accel);
        // Merged History
        auto &src_history = src.merged_history;
        for (auto itr = src_history.begin(); itr != src_history.end(); itr++) {
            dst.merged_history[itr->first] = itr->second * weight;
        }
    }
}
}

void MergePriorityMovements(const std::vector<MovementCtr>& b_ctrs,
                            MovementCtr& result_ctr) {
    int n_behaviors = b_ctrs.size();

    // Search max priorities
    std::vector<float> max_ps(N_BODY_PARTS, 0.f);   // priorities
    std::vector<int> max_p_idxs(N_BODY_PARTS, -1);  // indices
    for (int i = 0; i < n_behaviors; i++) {
        const MovementCtr& src = b_ctrs[i];
        UpdateMaxPriority(max_ps[0], max_p_idxs[0], src.body, i);
        UpdateMaxPriority(max_ps[1], max_p_idxs[1], src.head, i);
        UpdateMaxPriority(max_ps[2], max_p_idxs[2], src.eyes, i);
        UpdateMaxPriority(max_ps[3], max_p_idxs[3], src.eyelid, i);
        UpdateMaxPriority(max_ps[4], max_p_idxs[4], src.right_arm, i);
        UpdateMaxPriority(max_ps[5], max_p_idxs[5], src.left_arm, i);
        UpdateMaxPriority(max_ps[6], max_p_idxs[6], src.voice, i);
    }

    // Empty behavior
    MovementCtr dst;

    // Merge with max priorities
    if (max_p_idxs[0] != -1) dst.body = b_ctrs[max_p_idxs[0]].body;
    if (max_p_idxs[1] != -1) dst.head = b_ctrs[max_p_idxs[1]].head;
    if (max_p_idxs[2] != -1) dst.eyes = b_ctrs[max_p_idxs[2]].eyes;
    if (max_p_idxs[3] != -1) dst.eyelid = b_ctrs[max_p_idxs[3]].eyelid;
    if (max_p_idxs[4] != -1) dst.right_arm = b_ctrs[max_p_idxs[4]].right_arm;
    if (max_p_idxs[5] != -1) dst.left_arm = b_ctrs[max_p_idxs[5]].left_arm;
    if (max_p_idxs[6] != -1) dst.voice = b_ctrs[max_p_idxs[6]].voice;
    // Tag
    dst.tag = "P Merged";

    // Calculate merged weights (softmax)
    std::vector<float> sum_ps_exp(N_BODY_PARTS, 0.f);
    for (int i = 0; i < n_behaviors; i++) {
        const MovementCtr& src = b_ctrs[i];
        sum_ps_exp[0] += expf(src.body.w - max_ps[0]);
        sum_ps_exp[1] += expf(src.head.w - max_ps[1]);
        sum_ps_exp[2] += expf(src.eyes.w - max_ps[2]);
        sum_ps_exp[3] += expf(src.eyelid.w - max_ps[3]);
        sum_ps_exp[4] += expf(src.right_arm.w - max_ps[4]);
        sum_ps_exp[5] += expf(src.left_arm.w - max_ps[5]);
        sum_ps_exp[6] += expf(src.voice.w - max_ps[6]);
    }
    // exp(max_ps - max_ps) / sum(ps[i] - max_ps)
    dst.body.w = 1.f / sum_ps_exp[0];
    dst.head.w = 1.f / sum_ps_exp[1];
    dst.eyes.w = 1.f / sum_ps_exp[2];
    dst.eyelid.w = 1.f / sum_ps_exp[3];
    dst.right_arm.w = 1.f / sum_ps_exp[4];
    dst.left_arm.w = 1.f / sum_ps_exp[5];
    dst.voice.w = 1.f / sum_ps_exp[6];

    // Set to return
    result_ctr = dst;
}

void MergeWeightedMovements(const std::vector<MovementCtr>& b_ctrs,
                            MovementCtr& result_ctr) {
    int n_behaviors = b_ctrs.size();

    // Sum up weights
    std::vector<float> sum_ws(N_BODY_PARTS - 1, 0.f);  // weights except `voice`
    for (int i = 0; i < n_behaviors; i++) {
        const MovementCtr& src = b_ctrs[i];
        UpdateSumWeight(sum_ws[0], src.body);
        UpdateSumWeight(sum_ws[1], src.head);
        UpdateSumWeight(sum_ws[2], src.eyes);
        UpdateSumWeight(sum_ws[3], src.eyelid);
        UpdateSumWeight(sum_ws[4], src.right_arm);
        UpdateSumWeight(sum_ws[5], src.left_arm);
    }

    // Empty behavior
    MovementCtr dst;

    // Merge with weights
    for (int i = 0; i < n_behaviors; i++) {
        const MovementCtr& src = b_ctrs[i];
        AccumMovementPart(dst.body, src.body, sum_ws[0]);
        AccumMovementPart(dst.head, src.head, sum_ws[1]);
        AccumMovementPart(dst.eyes, src.eyes, sum_ws[2]);
        AccumMovementPart(dst.eyelid, src.eyelid, sum_ws[3]);
        AccumMovementPart(dst.right_arm, src.right_arm, sum_ws[4]);
        AccumMovementPart(dst.left_arm, src.left_arm, sum_ws[5]);
    }

    // Merge voice (priority)
    float max_p_voice = 0.f;  // priority
    int max_p_idx = -1;       // index
    for (int i = 0; i < n_behaviors; i++) {
        const MovementCtr& src = b_ctrs[i];
        UpdateMaxPriority(max_p_voice, max_p_idx, src.voice, i);
    }
    if (max_p_idx != -1) dst.voice = b_ctrs[max_p_idx].voice;

    // Set to return
    result_ctr = dst;
}
