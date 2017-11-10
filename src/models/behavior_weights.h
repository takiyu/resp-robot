#ifndef BEHAVIOR_WEIGHTS_H_161214
#define BEHAVIOR_WEIGHTS_H_161214
#include <dlib/matrix.h>
#include <string>
#include <vector>

#include "behavior_base.h"

// Declaration of Weight types
template <typename T>
using Maps = std::map<std::string, std::map<std::string, T> >;
using WeightMaps = Maps<float>;
using DlibColVec = dlib::matrix<double, 0, 1>;

// w_maps -> behavior_coll
void SetBehaviorWeightMaps(const WeightMaps& w_maps,
                           BehaviorCollection& behavior_coll);

// behavior_coll -> w_maps
void GetBehaviorWeightMaps(const BehaviorCollection& behavior_coll,
                           WeightMaps& w_maps);

// json -> p_maps, w_maps
bool LoadBehaviorWeights(const std::string& filename, WeightMaps& p_maps,
                         WeightMaps& w_maps);

// p_maps, w_maps -> json
bool SaveBehaviorWeights(const WeightMaps& p_maps, const WeightMaps& w_maps,
                         const std::string& filename);

// p_maps, w_maps <-> dlib::matrix
class BehaviorWeightsConverter {
public:
    BehaviorWeightsConverter() : n_idxs(0) {}
    ~BehaviorWeightsConverter() {}

    // Initialize conversion indices
    void initIdxs(const WeightMaps& p_maps, const WeightMaps& w_maps);
    // p_maps, w_maps -> dlib::matrix
    void convertToDlib(const WeightMaps& p_maps, const WeightMaps& w_maps,
                       DlibColVec& dst_vec) const;
    // dlib::matrix -> p_maps, w_maps
    void convertFromDlib(const DlibColVec& src_vec, WeightMaps& p_maps,
                         WeightMaps& w_maps) const;

    int p_size() { return n_idxs_p; }
    int w_size() { return n_idxs_w; }

private:
    int n_idxs, n_idxs_p, n_idxs_w;
    Maps<int> p_idx_maps, w_idx_maps;
};

#endif
