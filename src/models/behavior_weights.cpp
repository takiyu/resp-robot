#include "behavior_weights.h"

#include <fstream>
#include <sstream>

#include "picojson.h"

namespace {
const std::string P_JSON_KEY = "Priority Behaviors";
const std::string W_JSON_KEY = "Weighted Behaviors";

bool loadFileLines(const std::string& filename, std::string& lines) {
    std::ifstream in_stream(filename.c_str(), std::ios::in);
    if (!in_stream.is_open()) {
        return false;
    }
    std::stringstream lines_ss;
    lines_ss << in_stream.rdbuf();
    in_stream.close();
    lines = lines_ss.str();
    return true;
}

bool loadJsonLines(const std::string& filename, picojson::value& v) {
    // Load json file
    std::string file_lines;
    if (!loadFileLines(filename, file_lines)) {
        printf(" >> Failed to load file\n");
        return false;
    }

    // Parse json lines
    std::string err = picojson::parse(v, file_lines);
    if (!err.empty()) {
        printf(" >> Failed to parse the json\n");
        return false;
    }

    return true;
}

// Parse json map
template <typename T, typename T_INTERNAL>
void parseJsonValues(const std::map<std::string, picojson::value>& src_json,
                     std::map<std::string, T>& dst_map) {
    dst_map.clear();
    for (auto itr = src_json.begin(); itr != src_json.end(); itr++) {
        dst_map[itr->first] = itr->second.get<T_INTERNAL>();
    }
}

// Parse json map (Recursive call)
template <typename T, typename T_INTERNAL>
void parseJsonValues(const std::map<std::string, picojson::value>& src_json,
                     Maps<T>& dst_map) {
    dst_map.clear();
    for (auto itr = src_json.begin(); itr != src_json.end(); itr++) {
        parseJsonValues<T, T_INTERNAL>(itr->second.get<picojson::object>(),
                                       dst_map[itr->first]);
    }
}

// Create json map
template <typename T>
void createJsonValues(const std::map<std::string, T>& src_map,
                      std::map<std::string, picojson::value>& dst_json) {
    dst_json.clear();
    for (auto itr = src_map.begin(); itr != src_map.end(); itr++) {
        dst_json[itr->first] = picojson::value(itr->second);
    }
}

// Create json map (Recursive call)
template <typename T>
void createJsonValues(const Maps<T>& src_map,
                      std::map<std::string, picojson::value>& dst_json) {
    dst_json.clear();
    for (auto itr = src_map.begin(); itr != src_map.end(); itr++) {
        std::map<std::string, picojson::value> tmp_json;
        createJsonValues(itr->second, tmp_json);
        dst_json[itr->first] = picojson::value(tmp_json);
    }
}

// Assign index count to the map
int fillWeightIndicesMap(const WeightMaps& w_maps, Maps<int>& idx_maps,
                         int idx_cnt) {
    // Loop for weight maps
    for (auto b_itr = w_maps.begin(); b_itr != w_maps.end(); b_itr++) {
        const std::map<std::string, float>& w_map = b_itr->second;
        for (auto m_itr = w_map.begin(); m_itr != w_map.end(); m_itr++) {
            if (idx_maps[b_itr->first].count(m_itr->first) != 0) {
                printf(">> Duplicated weight keys (%s / %s)\n",
                       b_itr->first.c_str(), m_itr->first.c_str());
                continue;
            }
            // Assign
            idx_maps[b_itr->first][m_itr->first] = idx_cnt;
            idx_cnt++;
        }
    }
    return idx_cnt;
}

// Convert maps of weights to dlib vector (dlib vector must be allocated)
void cvtWeightMaps2Vec(const WeightMaps& src_maps, DlibColVec& dst_vec,
                       const Maps<int>& idx_maps) {
    // Loop for index maps
    for (auto b_itr = idx_maps.begin(); b_itr != idx_maps.end(); b_itr++) {
        const std::map<std::string, int>& idx_map = b_itr->second;
        for (auto m_itr = idx_map.begin(); m_itr != idx_map.end(); m_itr++) {
            if (src_maps.count(b_itr->first) == 0 ||
                src_maps.at(b_itr->first).count(m_itr->first) == 0) {
                printf(">> Lack of a weight key (%s / %s)\n",
                       b_itr->first.c_str(), m_itr->first.c_str());
                continue;
            }
            // Assign weight value to dlib vector
            int idx = m_itr->second;
            if (dst_vec.size() <= idx) {
                printf(">> Index out of range (%d, %d)\n", idx,
                       static_cast<int>(dst_vec.size()));
                continue;
            }
            dst_vec(idx) = src_maps.at(b_itr->first).at(m_itr->first);
        }
    }
}

// Convert maps of weights from dlib vector
void cvtWeightVec2Maps(const DlibColVec& src_vec, WeightMaps& dst_maps,
                       const Maps<int>& idx_maps) {
    // Loop for index maps
    for (auto b_itr = idx_maps.begin(); b_itr != idx_maps.end(); b_itr++) {
        const std::map<std::string, int>& idx_map = b_itr->second;
        for (auto m_itr = idx_map.begin(); m_itr != idx_map.end(); m_itr++) {
            if (dst_maps.count(b_itr->first) != 0 &&
                dst_maps.at(b_itr->first).count(m_itr->first) != 0) {
                printf(">> Duplicated weight keys (%s / %s)\n",
                       b_itr->first.c_str(), m_itr->first.c_str());
                continue;
            }
            // Assign weight value from dlib vector
            int idx = m_itr->second;
            if (src_vec.size() <= idx) {
                printf(">> Index out of range (%d, %d)\n", idx,
                       static_cast<int>(src_vec.size()));
                continue;
            }
            dst_maps[b_itr->first][m_itr->first] = src_vec(idx);
        }
    }
}

}  // namespace

// w_maps -> behavior_coll
void SetBehaviorWeightMaps(const WeightMaps& w_maps,
                           BehaviorCollection& behavior_coll) {
    // For all behaviors
    int n_behaviors = behavior_coll.size();
    for (int i = 0; i < n_behaviors; i++) {
        // Check lack of a tag in the w_maps
        const std::string TAG = behavior_coll.tag(i);
        if (w_maps.count(TAG) == 0) {
            printf(" >> Lack of a behavior tag (%s)\n", TAG.c_str());
            continue;
        }
        // Set to behavior_coll
        const std::map<std::string, float>& w_map = w_maps.at(TAG);
        for (auto w_itr = w_map.begin(); w_itr != w_map.end(); w_itr++) {
            behavior_coll[i]->setWeight(w_itr->first, w_itr->second);
        }
    }
}

// behavior_coll -> w_maps
void GetBehaviorWeightMaps(const BehaviorCollection& behavior_coll,
                           WeightMaps& w_maps) {
    w_maps.clear();
    // For all behaviors
    int n_behaviors = behavior_coll.size();
    for (int i = 0; i < n_behaviors; i++) {
        // Check tag duplication in behaviors
        const std::string TAG = behavior_coll.tag(i);
        if (w_maps.count(TAG) != 0) {
            printf(" >> Duplicated behavior tags (%s)\n", TAG.c_str());
            continue;
        }
        // Set to w_maps
        w_maps[TAG] = behavior_coll[i]->getWeightMap();
    }
}

// json -> p_maps, w_maps
bool LoadBehaviorWeights(const std::string& filename, WeightMaps& p_maps,
                         WeightMaps& w_maps) {
    // Load json file
    printf("* Load behavior weights: %s\n", filename.c_str());
    picojson::value v;
    if (!loadJsonLines(filename, v)) {
        return false;
    }

    // Json root object
    std::map<std::string, picojson::value> json_root =
        v.get<picojson::object>();

    // Get json values of priorities and weights
    if (json_root.count(P_JSON_KEY) == 0) {
        printf(" >> No priorities\n");
        return false;
    }
    if (json_root.count(W_JSON_KEY) == 0) {
        printf(" >> No weights\n");
        return false;
    }
    std::map<std::string, picojson::value> json_priorities =
        json_root[P_JSON_KEY].get<picojson::object>();
    std::map<std::string, picojson::value> json_weights =
        json_root[W_JSON_KEY].get<picojson::object>();

    // Parse json value maps to float maps
    parseJsonValues<float, double>(json_priorities, p_maps);
    parseJsonValues<float, double>(json_weights, w_maps);

    return true;
}

// p_maps, w_maps -> json
bool SaveBehaviorWeights(const WeightMaps& p_maps, const WeightMaps& w_maps,
                         const std::string& filename) {
    printf("* Save behavior weights: %s\n", filename.c_str());

    // Create json values
    std::map<std::string, picojson::value> json_priorities;
    std::map<std::string, picojson::value> json_weights;
    createJsonValues(p_maps, json_priorities);
    createJsonValues(w_maps, json_weights);

    // Merge jsons of priorities and weights
    std::map<std::string, picojson::value> json_root;
    json_root[P_JSON_KEY] = picojson::value(json_priorities);
    json_root[W_JSON_KEY] = picojson::value(json_weights);

    // Convert to single json value
    picojson::value json(json_root);

    // Write to a file
    std::ofstream out_stream(filename.c_str(), std::ios::out);
    if (!out_stream.is_open()) {
        return false;
    }
    out_stream << json.serialize(true);  // prettify is true to read
    out_stream.close();

    return true;
}

// p_maps, w_maps <-> dlib::matrix
void BehaviorWeightsConverter::initIdxs(const WeightMaps& p_maps,
                                        const WeightMaps& w_maps) {
    p_idx_maps.clear();
    w_idx_maps.clear();
    this->n_idxs_p = fillWeightIndicesMap(p_maps, this->p_idx_maps, 0);
    this->n_idxs = fillWeightIndicesMap(w_maps, this->w_idx_maps,
                                        this->n_idxs_p);
    this->n_idxs_w = this->n_idxs - this->n_idxs_p;
}

void BehaviorWeightsConverter::convertToDlib(const WeightMaps& p_maps,
                                             const WeightMaps& w_maps,
                                             DlibColVec& dst_vec) const {
    // Allocate
    dst_vec.set_size(this->n_idxs);
    // Copy
    cvtWeightMaps2Vec(p_maps, dst_vec, this->p_idx_maps);
    cvtWeightMaps2Vec(w_maps, dst_vec, this->w_idx_maps);
}

void BehaviorWeightsConverter::convertFromDlib(const DlibColVec& src_vec,
                                               WeightMaps& p_maps,
                                               WeightMaps& w_maps) const {
    // Clear
    p_maps.clear();
    w_maps.clear();
    // Copy
    cvtWeightVec2Maps(src_vec, p_maps, this->p_idx_maps);
    cvtWeightVec2Maps(src_vec, w_maps, this->w_idx_maps);
}
