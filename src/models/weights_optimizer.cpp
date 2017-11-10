#include "weights_optimizer.h"

#include <chrono>

#include <dlib/optimization.h>
#include <glm/gtc/type_ptr.hpp>

#include "imgui/imgui.h"
#include "picojson.h"

namespace {
inline void sleepMillisec(float ms) {
    boost::this_thread::sleep(boost::posix_time::milliseconds(ms));
}

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

// === Json parsings functions ===
template <typename T>
inline void parseJsonValue(T& dst, picojson::value& json) {
    dst = static_cast<T>(json.get<double>());
}

inline void parseJsonValue(bool& dst, picojson::value& json) {
    dst = json.get<bool>();
}

inline void parseJsonValue(std::string& dst, picojson::value& json) {
    dst = json.get<std::string>();
}

inline void parseJsonValue(glm::vec3& dst, picojson::value& json) {
    picojson::array& json_arr = json.get<picojson::array>();
    float x = static_cast<float>(json_arr[0].get<double>());
    float y = static_cast<float>(json_arr[1].get<double>());
    float z = static_cast<float>(json_arr[2].get<double>());
    dst = glm::vec3(x, y, z);
}

inline void parseJsonValue(std::pair<glm::vec3, glm::vec3>& dst,
                           picojson::value& json) {
    picojson::array& json_arr = json.get<picojson::array>();
    parseJsonValue(dst.first, json_arr[0]);
    parseJsonValue(dst.second, json_arr[1]);
}

// parse attention indices
void parseAttensionIndices(CameraScene::AttentionIdx &dst,
                           const std::string &att_str) {
    if (att_str == "robovie") {
        dst = CameraScene::AttentionIdx::ROBOVIE;
    } else {
        for (int i = 0; i < N_OBJECT_DETECT; i++) {
            int obj_idx = CameraScene::AttentionIdx::OBJECT + i;
            std::stringstream ss;
            ss << "object" << i;
            if (att_str == ss.str()) {
                dst = static_cast<CameraScene::AttentionIdx>(obj_idx);
                return;
            }
        }
    }
    printf(" >> Invalid attention string (%s)\n", att_str.c_str());
}

// parse sensor json
void parseSensorContainer(picojson::object& json_sensor_obj,
                          SensorCtr& sensor_ctr) {
    std::map<std::string, picojson::value>& json_sensor = json_sensor_obj;
    for (auto itr = json_sensor.begin(); itr != json_sensor.end(); itr++) {
        std::string key = itr->first;

        // Face ("face": { "idx": 0, "pos": [0, 0, 100] })
        if (key == "face") {
            // Parse value to object
            std::map<std::string, picojson::value> json =
                itr->second.get<picojson::object>();
            // Index position and attention
            if (json.count("idx") > 0) {
                // Idx
                int idx;
                parseJsonValue(idx, json["idx"]);
                // Allocate
                int min_size = idx + 1;
                if (sensor_ctr.face_valids.size() < min_size) {
                    sensor_ctr.face_valids.resize(min_size, false);
                    sensor_ctr.face_poses.resize(min_size, glm::vec3(0, 0, 0));
                    auto att_none = CameraScene::AttentionIdx::NONE;
                    sensor_ctr.att_idxs.resize(min_size, att_none);
                    sensor_ctr.last_att_idxs.resize(min_size, att_none);
                    sensor_ctr.last_obj_att_idxs.resize(min_size, att_none);
                }
                sensor_ctr.face_valids[idx] = true;
                // Pos
                glm::vec3 pos;
                parseJsonValue(pos, json["pos"]);
                sensor_ctr.face_poses[idx] = pos;
                // Attention
                std::string att_str;
                if (json.count("attention")) {
                    parseJsonValue(att_str, json["attention"]);
                    parseAttensionIndices(sensor_ctr.att_idxs[idx], att_str);
                }
                // Last attention
                std::string last_att_str;
                if (json.count("last_attention")) {
                    parseJsonValue(last_att_str, json["last_attention"]);
                    parseAttensionIndices(sensor_ctr.last_att_idxs[idx],
                                          last_att_str);
                }
                // Last object attention
                std::string last_obj_att_str;
                if (json.count("last_obj_attention")) {
                    parseJsonValue(last_obj_att_str,
                                   json["last_obj_attention"]);
                    parseAttensionIndices(sensor_ctr.last_obj_att_idxs[idx],
                                          last_obj_att_str);
                }
            }
        }

        // Object ("object": { "idx": 0, "pos": [0, 0, 50] })
        else if (key == "object") {
            // Parse value to object
            std::map<std::string, picojson::value> json =
                itr->second.get<picojson::object>();
            // Index and position
            int idx;
            glm::vec3 pos;
            parseJsonValue(idx, json["idx"]);
            parseJsonValue(pos, json["pos"]);
            // Allocate
            int min_size = idx + 1;
            if (sensor_ctr.object_valids.size() < min_size) {
                sensor_ctr.object_valids.resize(min_size, false);
                sensor_ctr.object_poses.resize(min_size, glm::vec3(0, 0, 0));
            }
            // Register
            sensor_ctr.object_valids[idx] = true;
            sensor_ctr.object_poses[idx] = pos;
        }

        // VAD ("vad": { "active": true, "started": true, "finished": true,
        //               "len_ms": 1.0 })
        else if (key == "vad") {
            // Parse value to object
            std::map<std::string, picojson::value> json =
                itr->second.get<picojson::object>();
            // Active
            if (json.count("active")) {
                bool active;
                parseJsonValue(active, json["active"]);
                sensor_ctr.voice_active = active;
            }
            // Started
            if (json.count("started")) {
                bool started;
                parseJsonValue(started, json["started"]);
                sensor_ctr.voice_started = started;
            }
            // Finished
            if (json.count("finished")) {
                bool finished;
                parseJsonValue(finished, json["finished"]);
                sensor_ctr.voice_finished = finished;
            }
            // Length
            if (json.count("len_ms")) {
                float len_ms;
                parseJsonValue(len_ms, json["len_ms"]);
                sensor_ctr.voice_len_ms = len_ms;
            }
        }

        // Voice Recog ("voice_recog": { "processing": true, "recoged": true,
        //                               "result": "hello" }
        else if (key == "voice_recog") {
            // Parse value to object
            std::map<std::string, picojson::value> json =
                itr->second.get<picojson::object>();
            // Values
            bool processing, recoged, result;
            parseJsonValue(processing, json["processing"]);
            parseJsonValue(recoged, json["recoged"]);
            parseJsonValue(result, json["result"]);
            // Register
            sensor_ctr.voice_recog_processing = processing;
            sensor_ctr.voice_recoged = recoged;
            sensor_ctr.voice_recog_result = result;
        }
    }
}

// parse behavior json (part)
template <typename T>
void parseBehaviorContainerPart(MovementEvaluationCtr::BasePart<T>& dst,
                                picojson::object& json_obj) {
    std::map<std::string, picojson::value>& json = json_obj;
    for (auto itr = json.begin(); itr != json.end(); itr++) {
        std::string key = itr->first;
        // Parse value or percentages
        if (key == "val") {
            // Value
            dst.v.first = true;
            parseJsonValue(dst.v.second, itr->second);
        } else if (key == "per") {
            // Percentage
            std::map<std::string, picojson::value>& json_per =
                itr->second.get<picojson::object>();
            for (auto per_itr = json_per.begin(); per_itr != json_per.end();
                 per_itr++) {
                // Set percentages
                double percentage = per_itr->second.get<double>();
                dst.behavior_pers[per_itr->first] = percentage;
            }
        }
    }
}

// parse behavior json
void parseBehaviorContainer(picojson::object& json_behavior_obj,
                            MovementEvaluationCtr& behavior_ctr) {
    std::map<std::string, picojson::value>& json_behavior = json_behavior_obj;
    for (auto itr = json_behavior.begin(); itr != json_behavior.end(); itr++) {
        std::string key = itr->first;
        picojson::object json_obj = itr->second.get<picojson::object>();
        // Parse each part
        if (key == "body") {
            parseBehaviorContainerPart(behavior_ctr.body, json_obj);
        } else if (key == "head") {
            parseBehaviorContainerPart(behavior_ctr.head, json_obj);
        } else if (key == "eyes") {
            parseBehaviorContainerPart(behavior_ctr.eyes, json_obj);
        } else if (key == "eyelid") {
            parseBehaviorContainerPart(behavior_ctr.eyelid, json_obj);
        } else if (key == "right_arm") {
            parseBehaviorContainerPart(behavior_ctr.right_arm, json_obj);
        } else if (key == "left_arm") {
            parseBehaviorContainerPart(behavior_ctr.left_arm, json_obj);
        } else if (key == "voice") {
            parseBehaviorContainerPart(behavior_ctr.voice, json_obj);
        }
    }
}

// Debug Prints
std::ostream& operator<<(std::ostream& os, const glm::vec3& v) {
    os << "[" << v[0] << ", " << v[1] << ", " << v[2] << "]";
}

std::ostream& operator<<(std::ostream& os,
                         const std::pair<glm::vec3, glm::vec3>& v) {
    os << "[ " << v.first << ", " << v.second << " ]";
}

void printSensor(const SensorCtr& s) {
    // Face valids
    std::cout << "   >> face valids: ";
    for (int i = 0; i < s.face_valids.size(); i++) {
        std::cout << static_cast<int>(s.face_valids[i]) << ", ";
    }
    std::cout << std::endl;
    // Face positions
    std::cout << "   >> face poses: ";
    for (int i = 0; i < s.face_poses.size(); i++) {
        std::cout << s.face_poses[i] << ", ";
    }
    std::cout << std::endl;
    // AttentionIdx
    std::cout << "   >> attension indices: ";
    for (int i = 0; i < s.att_idxs.size(); i++) {
        std::cout << s.att_idxs[i] << ", ";
    }
    std::cout << std::endl;
    std::cout << "   >> last attension indices: ";
    for (int i = 0; i < s.last_att_idxs.size(); i++) {
        std::cout << s.last_att_idxs[i] << ", ";
    }
    std::cout << std::endl;
    std::cout << "   >> last obj attension indices: ";
    for (int i = 0; i < s.last_obj_att_idxs.size(); i++) {
        std::cout << s.last_obj_att_idxs[i] << ", ";
    }
    std::cout << std::endl;
    // Object valids
    std::cout << "   >> object valids: ";
    for (int i = 0; i < s.object_valids.size(); i++) {
        std::cout << static_cast<int>(s.object_valids[i]) << ", ";
    }
    std::cout << std::endl;
    // Object positions
    std::cout << "   >> object poses: ";
    for (int i = 0; i < s.object_poses.size(); i++) {
        std::cout << s.object_poses[i] << ", ";
    }
    std::cout << std::endl;
    // Voice
    std::cout << "   >> voice activie: " << static_cast<int>(s.voice_active)
              << std::endl;
    std::cout << "   >> voice started: " << static_cast<int>(s.voice_started)
              << std::endl;
    std::cout << "   >> voice finished: " << static_cast<int>(s.voice_finished)
              << std::endl;
    std::cout << "   >> voice len ms: " << s.voice_len_ms << std::endl;
    std::cout << "   >> voice recog processing: "
              << static_cast<int>(s.voice_recog_processing) << std::endl;
    std::cout << "   >> voice recoged: " << static_cast<int>(s.voice_recoged)
              << std::endl;
    std::cout << "   >> voice recog result: " << s.voice_recog_result
              << std::endl;
}

template <typename T>
void printBehavior(const MovementEvaluationCtr::BasePart<T>& part,
                   const std::string& name) {
    bool valid_value = part.v.first;
    bool valid_percentages = !part.behavior_pers.empty();
    if (valid_value || valid_percentages) {
        printf("   >[%s]\n", name.c_str());
    }
    if (valid_value) {
        printf("    > value: ");
        std::cout << part.v.second << std::endl;
    }
    if (valid_percentages) {
        printf("    > percentages:\n");
        auto &pers = part.behavior_pers;
        for (auto itr = pers.begin(); itr != pers.end(); itr++) {
            printf("     >> %s: %f\n", itr->first.c_str(), itr->second);
        }
    }
}

void printScenarios(
    const std::map<std::string,
                   std::vector<BehaviorWeightsOptimizer::ScenarioElem> >&
        scenarios) {
    printf("* Print Scenarios\n");
    for (auto itr = scenarios.begin(); itr != scenarios.end(); itr++) {
        printf(" >> %s \n", itr->first.c_str());
        const auto& elems = itr->second;
        for (int i = 0; i < elems.size(); i++) {
            // Sensor
            printf("  > Sensor:\n");
            printSensor(elems[i].sensor);

            // Behavior
            printf("  > Behavior:\n");
            const MovementEvaluationCtr& b = elems[i].behavior;
            printBehavior(b.body, "body");
            printBehavior(b.head, "head");
            printBehavior(b.eyes, "eyes");
            printBehavior(b.eyelid, "eyelid");
            printBehavior(b.left_arm, "left_arm");
            printBehavior(b.right_arm, "right_arm");
            printBehavior(b.voice, "voice");

            // Importance
            printf("  > Period_ms:  %f\n", elems[i].period_ms);
            printf("  > Importance: %f\n", elems[i].importance);
            printf("\n");
        }
        printf("\n");
    }
}

// Evaluation
double evaluateValue(const float a, const float b) {
    float diff = a - b;
    return diff * diff;
}
double evaluateValue(const glm::vec3& a, const glm::vec3& b) {
    return evaluateValue(a[0], b[0]) + evaluateValue(a[1], b[1]) +
           evaluateValue(a[2], b[2]);
}
double evaluateValue(const std::string& a, const std::string& b) {
    if (a == b) return 0.0;
    else return 1.0;
}
double evaluateValue(const std::pair<glm::vec3, glm::vec3>& a,
                     const std::pair<glm::vec3, glm::vec3>& b) {
    return evaluateValue(a.first, b.first) + evaluateValue(a.second, b.second);
}

template <typename T>
double evaluateBehaviorPart(const MovementCtr::BasePart<T>& result,
                            const MovementEvaluationCtr::BasePart<T>& eval,
                            float v_coeff, float p_coeff,
                            const std::string& tag, bool print) {
    auto &eval_pers = eval.behavior_pers;
    if (print && (eval.v.first || !eval_pers.empty())) {
        std::cout << "  > [" << tag << "]" << std::endl;
    }

    double error = 0.0;

    // Value
    if (eval.v.first) {
        const T &eval_v = eval.v.second;
        const T &res_v = result.v;
        // Evaluate value difference
        error += evaluateValue(eval_v, res_v) * result.w * v_coeff;
        // Print
        if (print) {
            std::cout << "   >> [values]  cur:" << res_v
                      << ", dst:" << eval_v << std::endl;
        }
    }

    // Percentages
    if (print && !eval_pers.empty()) {
        std::cout << "   >> [percentages]:" << std::endl;
    }
    for (auto itr = eval_pers.begin(); itr != eval_pers.end(); itr++) {
        const std::string &eval_b_tag = itr->first;
        const float eval_b_per = itr->second;
        const float res_b_per = result.merged_history.count(eval_b_tag) ?
            result.merged_history.at(eval_b_tag) : 0.f;
        float diff_per = eval_b_per - res_b_per;
        error += diff_per * diff_per * p_coeff;
        // Print
        if (print) {
            std::cout << "    " << eval_b_tag << "  cur:" << res_b_per
                      << ", dst:" << eval_b_per<< std::endl;
        }
    }

    return error;
}

double evaluateBehavior(const MovementCtr& res,
                        const MovementEvaluationCtr& eval, float p_coeff,
                        float v_coeff, bool print) {
    double error = 0.0;
    error += evaluateBehaviorPart(res.body, eval.body, p_coeff, v_coeff,
                                  "body", print);
    error += evaluateBehaviorPart(res.head, eval.head, p_coeff, v_coeff,
                                  "head", print);
    error += evaluateBehaviorPart(res.eyes, eval.eyes, p_coeff, v_coeff,
                                  "eyes", print);
    error += evaluateBehaviorPart(res.eyelid, eval.eyelid, p_coeff, v_coeff,
                                  "eyelid", print);
    error += evaluateBehaviorPart(res.left_arm, eval.left_arm, p_coeff, v_coeff,
                                  "left_arm", print);
    error += evaluateBehaviorPart(res.right_arm, eval.right_arm, p_coeff,
                                  v_coeff, "right_arm", print);
    error += evaluateBehaviorPart(res.voice, eval.voice, p_coeff, v_coeff,
                                  "voice", print);
    return error;
}

// Merge
void MergeEmptyMovement(const MovementCtr& src, MovementCtr& dst) {
    if (dst.body.empty) dst.body = src.body;
    if (dst.head.empty) dst.head = src.head;
    if (dst.eyes.empty) dst.eyes = src.eyes;
    if (dst.eyelid.empty) dst.eyelid = src.eyelid;
    if (dst.left_arm.empty) dst.left_arm = src.left_arm;
    if (dst.right_arm.empty) dst.right_arm = src.right_arm;
    if (dst.voice.empty) dst.voice = src.voice;
}

}  // namespace

void BehaviorWeightsOptimizer::init() { resp_model.init(); }

void BehaviorWeightsOptimizer::exit() { resp_model.exit(); }

bool BehaviorWeightsOptimizer::loadScenarios(const std::string& filename) {
    // Load json file
    printf("* Load scenario json: %s\n", filename.c_str());
    picojson::value v;
    if (!loadJsonLines(filename, v)) {
        return false;
    }

    // Json root object
    std::map<std::string, picojson::value> json_root =
        v.get<picojson::object>();

    // Parse scenario arrays
    std::map<std::string, picojson::value> json_sce_arrs;
    for (auto itr = json_root.begin(); itr != json_root.end(); itr++) {
        json_sce_arrs[itr->first] = itr->second;
    }

    // For each element of the scenario array
    for (auto itr = json_sce_arrs.begin(); itr != json_sce_arrs.end(); itr++) {
        std::string sce_name = itr->first;
        picojson::array& json_sce_arr = itr->second.get<picojson::array>();

        // Create unique scenario key
        while (this->scenarios.count(sce_name) != 0) {
            sce_name = sce_name + "_";
        }

        // Register new scenario
        std::vector<ScenarioElem>& dst_sce = this->scenarios[sce_name];
        for (int i = 0; i < json_sce_arr.size(); i++) {
            std::map<std::string, picojson::value>& json_sce_elem =
                json_sce_arr[i].get<picojson::object>();
            // Check the validness
            if (json_sce_elem.count("sensor") == 0 ||
                json_sce_elem.count("behavior") == 0 ||
                json_sce_elem.count("period_ms") == 0) {
                printf(" >> Invalid scenario element (%s[%d])\n",
                       sce_name.c_str(), i);
                continue;
            }

            // Sensor
            SensorCtr sensor_ctr;
            picojson::object& json_sensor =
                json_sce_elem["sensor"].get<picojson::object>();
            parseSensorContainer(json_sensor, sensor_ctr);

            // Behavior
            std::map<std::string, picojson::value>& json_behavior =
                json_sce_elem["behavior"].get<picojson::object>();
            MovementEvaluationCtr behavior_ctr;
            parseBehaviorContainer(json_behavior, behavior_ctr);

            // Period_ms
            double period_ms = json_sce_elem["period_ms"].get<double>();

            // Importance
            double importance = 1.0;  // default importance
            if (json_sce_elem.count("importance") > 0) {
                importance = json_sce_elem["importance"].get<double>();
            }

            // Scenario element
            ScenarioElem sce_elem;
            sce_elem.sensor = sensor_ctr;
            sce_elem.behavior = behavior_ctr;
            sce_elem.period_ms = static_cast<float>(period_ms);
            sce_elem.importance = static_cast<float>(importance);

            // Register
            dst_sce.push_back(sce_elem);
        }
    }

    return true;
}

double BehaviorWeightsOptimizer::optimize() {
    printf("* Start behavior weights optimization\n");
    if (scenarios.empty()) {
        printf(" >> no scenario\n");
        return 0.0;
    }

    // Create starting_point
    DlibColVec starting_point;
    resp_model.getWeights(starting_point);
    DlibColVec org_starting_point = starting_point;

    // Optimize
    resp_model.setTimeScale(TIME_SCALE);  // Enable fast mode
    double error = dlib::find_min_using_approximate_derivatives(
        dlib::bfgs_search_strategy(), dlib::objective_delta_stop_strategy(1e-7),
        *this, starting_point, -1, 0.001);
    printf(" >> final error: %f\n", error);
    // Evaluation
    evaluateOne(starting_point, true);
    // Reset the model
    resp_model.setTimeScale(1.f);  // Disable fast mode

    // Set optimized value
    resp_model.setWeights(starting_point);

    optimized = true;

    return error;
}

double BehaviorWeightsOptimizer::evaluateOne(const DlibColVec& dlib_vec,
                                             bool print_eval) const {
    if (scenarios.empty()) {
        printf(" >> no scenario\n");
        return 0.0;
    }

    // // Backup original weights
    // DlibColVec org_dlib_vec;
    // resp_model.getWeights(org_dlib_vec);

    // Set weights to the model
    resp_model.setWeights(dlib_vec);

    // Evaluate with all scenarios
    double sum_error = 0.0;
    double sum_error_denom = 0.0;
    for (auto itr = scenarios.begin(); itr != scenarios.end(); itr++) {
        // Get time-series scenarios
        const std::vector<ScenarioElem> &sce_elems = itr->second;
        for (int i = 0; i < sce_elems.size(); i++) {
            if (print_eval) {
                std::cout << "  --- Evaluation ('" << itr->first << "' [" << i
                          << "]) ---" << std::endl;
            }

            // Get current scenario element
            const ScenarioElem &sce_elem = sce_elems[i];
            float period_ms = sce_elem.period_ms;
            float importance = sce_elem.importance;

            // Set sensor value
            resp_model.setOverwriteSensor(sce_elem.sensor);
            resp_model.update(true);  // set only

            MovementCtr result_behavior_ctr;

            // Update loop for a period
            std::chrono::system_clock::time_point start_clock =
                std::chrono::system_clock::now();
            while (true) {
                sleepMillisec(WAIT_RESP_MS);

                // Update
                resp_model.update();
                // Get result behavior
                MovementCtr tmp_behavior_ctr;
                resp_model.getResultBehavior(tmp_behavior_ctr);
                // Commit to final result
                MergeEmptyMovement(tmp_behavior_ctr, result_behavior_ctr);

                // Check spent time
                std::chrono::system_clock::time_point cur_clock =
                    std::chrono::system_clock::now();
                float elapsed_ms =
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                                       cur_clock - start_clock) .count();
                if (period_ms <= elapsed_ms * TIME_SCALE) break;
            }

            // Clear sensor values
            resp_model.clearOverwriteSensor();

            // Evaluate the behavior
            double error = evaluateBehavior(result_behavior_ctr,
                                            sce_elem.behavior,
                                            v_coeff, p_coeff, print_eval);
            sum_error += error * importance;
            sum_error_denom += importance;

            if (print_eval) std::cout << std::endl;
        }
        resp_model.exitAllBehaviors();
    }

    double error = sum_error / sum_error_denom;
    printf(" >> error: %f\n", error);

    // // Restore original weights
    // resp_model.setWeights(org_dlib_vec);

    return error;
}

void BehaviorWeightsOptimizer::initGlUi() { resp_model.initGlUi(); }

void BehaviorWeightsOptimizer::drawGlUi() {
    resp_model.drawGlUi();

    ImGui::Begin("Optimizer");
    // Load scenario json
    ImGui::InputText("##scenario_load", load_filename, sizeof(load_filename));
    bool ret_clear = ImGui::Button("Clear scenarios");
    ImGui::SameLine();
    bool ret_load_new = ImGui::Button("Load scenarios (new)");
    ImGui::SameLine();
    bool ret_load_append = ImGui::Button("Load scenarios (append)");

    if (ret_clear || ret_load_new) {
        // Clear and set result text
        clearScenarios();
        scenario_ret_str = ">> clear";
    }
    if (ret_load_append || ret_load_new) {
        // Save and set result text
        if (loadScenarios(std::string(load_filename))) {
            scenario_ret_str = ">> succeeded";
        } else {
            scenario_ret_str = ">> failed";
        }
    }
    ImGui::SameLine();
    ImGui::Text("%s", scenario_ret_str.c_str());
    ImGui::Separator();

    // Start button
    if (ImGui::Button("Start Optimization")) {
        latest_error = optimize();
    }
    ImGui::SameLine();
    if (ImGui::Button("Start Optimization with base weights")) {
        resp_model.setBaseWeights();
        latest_error = optimize();
    }
    if (optimized) {
        ImGui::SameLine();
        ImGui::Text(">> Error: %f", latest_error);
    }

    // Coefficients
    ImGui::DragFloat("value error coeff", &v_coeff);
    ImGui::DragFloat("percentage error coeff", &p_coeff);

    // [DEBUG] Print scenario elements
    if (ImGui::Button("[DEBUG] Print scenarios")) {
        printScenarios(scenarios);
    }

    ImGui::End();
}
