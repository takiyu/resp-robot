#include "resp_model.h"

#include "imgui/imgui.h"

namespace {
// Create plotters
template <typename T>
void CreatePartPlotters(std::vector<T>& plotters, const std::string& prefix) {
    plotters.resize(N_BODY_PARTS);
    plotters[0].setTitle(prefix + "[Body]");
    plotters[1].setTitle(prefix + "[Head]");
    plotters[2].setTitle(prefix + "[Eyes]");
    plotters[3].setTitle(prefix + "[Eyelid]");
    plotters[4].setTitle(prefix + "[Right Arm]");
    plotters[5].setTitle(prefix + "[Left Arm]");
    plotters[6].setTitle(prefix + "[Voice]");
    // Parameters  TODO:Be configurable
    for (int i = 0; i < plotters.size(); i++) {
        plotters[i].setSize(300, 200);
        plotters[i].setHistorySec(3.f);
        plotters[i].setYRange(0.f, 10.f);
        plotters[i].setLineWidth(4.f);
    }
}

// Initialize plotters
template <typename T>
void InitPlottersGlUi(std::vector<T>& plotters) {
    for (int i = 0; i < plotters.size(); i++) {
        plotters[i].initGlUi();
    }
}

// Draw plotters
template <typename T>
void DrawPlottersGlUi(std::vector<T>& plotters) {
    // Two columns
    ImGui::Columns(2);
    for (int i = 0; i < plotters.size(); i++) {
        plotters[i].drawGlUi();
        if (i == plotters.size() / 2) {
            ImGui::NextColumn();
        }
    }
}

// Update plotters with new data
template <typename T>
void UpdatePartPlotters(std::vector<T>& plotters,
                        const std::vector<MovementCtr>& behavior_ctrs) {
    assert(plotters.size() == N_BODY_PARTS);
    for (int i = 0; i < behavior_ctrs.size(); i++) {
        const MovementCtr& b = behavior_ctrs[i];
        plotters[0].appendData(b.tag, b.body.w);
        plotters[1].appendData(b.tag, b.head.w);
        plotters[2].appendData(b.tag, b.eyes.w);
        plotters[3].appendData(b.tag, b.eyelid.w);
        plotters[4].appendData(b.tag, b.right_arm.w);
        plotters[5].appendData(b.tag, b.left_arm.w);
        plotters[6].appendData(b.tag, b.voice.w);
    }
}

}  // namespace

// ===== Basic methods =====
void ResponsiveModel::init() {
    if (inited) return;
    modules.init();

    // Register behaviors (delete in exit())

    // Telepresence Behaviors
    p_behavior_coll.add(new MoveHeadBehavior(MoveHeadBehavior::UP));
    p_behavior_coll.add(new MoveHeadBehavior(MoveHeadBehavior::DOWN));
    p_behavior_coll.add(new MoveHeadBehavior(MoveHeadBehavior::RIGHT));
    p_behavior_coll.add(new MoveHeadBehavior(MoveHeadBehavior::LEFT));

    // Basic behaviors
    p_behavior_coll.add(new SleepBehavior());
    p_behavior_coll.add(new NodBehavior());
    p_behavior_coll.add(new HandWaveBehavior(HandWaveBehavior::LEFT_RAND));
    p_behavior_coll.add(new HandWaveBehavior(HandWaveBehavior::RIGHT_RAND));
    p_behavior_coll.add(new HandWaveBehavior(HandWaveBehavior::BOTH_RAND));
    // p_behavior_coll.add(new HandWaveBehavior(HandWaveBehavior::LEFT_RESP));
    // p_behavior_coll.add(new HandWaveBehavior(HandWaveBehavior::RIGHT_RESP));
    // p_behavior_coll.add(new HandWaveBehavior(HandWaveBehavior::BOTH_RESP));
    p_behavior_coll.add(new HandHoldOutBehavior(HandHoldOutBehavior::LEFT));
    p_behavior_coll.add(new HandHoldOutBehavior(HandHoldOutBehavior::RIGHT));
    p_behavior_coll.add(new HandHoldOutBehavior(HandHoldOutBehavior::BOTH));
    p_behavior_coll.add(new LookatBehavior(LookatBehavior::RAND_FACE));
    // p_behavior_coll.add(new LookatBehavior(LookatBehavior::RAND_OBJ));
    p_behavior_coll.add(new LookatBehavior(LookatBehavior::CLOSEST_FACE));
    // p_behavior_coll.add(new LookatBehavior(LookatBehavior::CLOSEST_OBJ));
    // p_behavior_coll.add(new LookatBehavior(LookatBehavior::MOVING_FACE));
    // p_behavior_coll.add(new LookatBehavior(LookatBehavior::MOVING_OBJ));
    p_behavior_coll.add(new LookatBehavior(LookatBehavior::EYE_CONTACT));
    // p_behavior_coll.add(new LookatBehavior(LookatBehavior::JOINT_ATTENTION));
    p_behavior_coll.add(new LookBackAttentionBehavior());
    p_behavior_coll.add(new LookatBodyBehavior());
    // p_behavior_coll.add(new HelloVoiceBehavior());

    w_behavior_coll.add(new BlinkBehavior(BlinkBehavior::RAND));
    w_behavior_coll.add(new BlinkBehavior(BlinkBehavior::DYNAMIC));
    w_behavior_coll.add(new BreathBehavior());
    w_behavior_coll.add(new NeckHoldWithSaccades());
    w_behavior_coll.add(new PersonalSpaceBehavior());

    // Initialize
    p_behavior_coll.initAll();
    w_behavior_coll.initAll();

    // Initialize converter indices
    WeightMaps p_maps, w_maps;
    getWeights(p_maps, w_maps);
    weights_converter.initIdxs(p_maps, w_maps);

    inited = true;
}

void ResponsiveModel::exit() {
    if (!inited) return;

    // Delete behaviors
    p_behavior_coll.releaseAll();
    w_behavior_coll.releaseAll();

    modules.exit();
    inited = false;
}

void ResponsiveModel::update(bool set_only) {
    if (!inited) {
        printf(">> ResponsiveModel is not initialized\n");
        return;
    }
    modules.update();

    // Force update for fast mode
    if (time_scale > 1.f) {
        modules.robovie_action.updateTransformMatrixForce();
    }

    // Get sensor inputs
    SensorCtr sensor_ctr;
    sensors.get(sensor_ctr);

    // Overwrite sensor values (No robovie variables)
    if (do_overwrite_sensor) {
        // Camera Scene
        sensor_ctr.face_valids = overwrite_sensor.face_valids;
        sensor_ctr.object_valids = overwrite_sensor.object_valids;
        sensor_ctr.face_poses = overwrite_sensor.face_poses;
        sensor_ctr.object_poses = overwrite_sensor.object_poses;
        sensor_ctr.att_idxs = overwrite_sensor.att_idxs;
        sensor_ctr.last_att_idxs = overwrite_sensor.last_att_idxs;
        sensor_ctr.last_obj_att_idxs = overwrite_sensor.last_obj_att_idxs;
        // VAD
        sensor_ctr.voice_active = overwrite_sensor.voice_active;
        sensor_ctr.voice_started = overwrite_sensor.voice_started;
        sensor_ctr.voice_finished = overwrite_sensor.voice_finished;
        sensor_ctr.voice_len_ms = overwrite_sensor.voice_len_ms;
        // Voice Recognition
        sensor_ctr.voice_recog_processing =
            overwrite_sensor.voice_recog_processing;
        sensor_ctr.voice_recoged = overwrite_sensor.voice_recoged;
        sensor_ctr.voice_recog_result = overwrite_sensor.voice_recog_result;
    }

    // Set only and return
    if (set_only) {
        for (int i = 0; i < p_behavior_coll.size(); i++) {
            p_behavior_coll[i]->setSensor(sensor_ctr);
        }
        for (int i = 0; i < w_behavior_coll.size(); i++) {
            w_behavior_coll[i]->setSensor(sensor_ctr);
        }
        return;
    }

    // Priority
    MovementCtr p_result_behavior;
    {
        // Priority behaviors occur and output
        int n_behaviors = p_behavior_coll.size();
        std::vector<MovementCtr> p_behavior_ctrs(n_behaviors);
        for (int i = 0; i < n_behaviors; i++) {
            p_behavior_coll[i]->setSensor(sensor_ctr);
            p_behavior_coll[i]->getMovement(p_behavior_ctrs[i]);
        }
        // Plot
        UpdatePartPlotters(p_plotters, p_behavior_ctrs);
        // Merge
        MergePriorityMovements(p_behavior_ctrs, p_result_behavior);
    }

    // Weight
    MovementCtr w_result_behavior;
    {
        // Weighted behaviors occur and output
        int n_behaviors = w_behavior_coll.size();
        std::vector<MovementCtr> w_behavior_ctrs(n_behaviors);
        for (int i = 0; i < n_behaviors; i++) {
            w_behavior_coll[i]->setSensor(sensor_ctr);
            w_behavior_coll[i]->getMovement(w_behavior_ctrs[i]);
        }
        // Add p behavior
        w_behavior_ctrs.push_back(p_result_behavior);
        // Plot
        UpdatePartPlotters(w_plotters, w_behavior_ctrs);
        // Merge
        MergeWeightedMovements(w_behavior_ctrs, w_result_behavior);
    }

    // [DEBUG] Print merged result (right_arm)
//     auto &history = w_result_behavior.right_arm.merged_history;
//     printf("> Merged result behavior (right_arm)\n");
//     for (auto itr = history.begin(); itr != history.end() ; itr++) {
//         printf(" %s %f\n", itr->first.c_str(), itr->second);
//     }
//     printf("\n");

    // Interrupt for result behavior cache
    cache_result_behavior = w_result_behavior;

    // Project to Robovie
    if (out_projection) {
        projector.project(w_result_behavior);
    }
}

void ResponsiveModel::setWeights(const WeightMaps& p_maps,
                                 const WeightMaps& w_maps) {
    SetBehaviorWeightMaps(p_maps, p_behavior_coll);
    SetBehaviorWeightMaps(w_maps, w_behavior_coll);
}

void ResponsiveModel::getWeights(WeightMaps& p_maps, WeightMaps& w_maps) {
    GetBehaviorWeightMaps(p_behavior_coll, p_maps);
    GetBehaviorWeightMaps(w_behavior_coll, w_maps);
}

void ResponsiveModel::setWeights(const DlibColVec& dlib_vec) {
    WeightMaps p_maps, w_maps;
    weights_converter.convertFromDlib(dlib_vec, p_maps, w_maps);
    setWeights(p_maps, w_maps);
}

void ResponsiveModel::getWeights(DlibColVec& dlib_vec) {
    WeightMaps p_maps, w_maps;
    getWeights(p_maps, w_maps);
    weights_converter.convertToDlib(p_maps, w_maps, dlib_vec);
}

void ResponsiveModel::setBaseWeights() {
    printf("* Set base weights\n");
    DlibColVec dlib_vec;
    getWeights(dlib_vec);
    int p_size = weights_converter.p_size();
    int w_size = weights_converter.w_size();
    for (int i = 0; i < p_size; i++) {
        dlib_vec(i) = 1.0;
    }
    for (int i = 0; i < w_size; i++) {
        dlib_vec(i + p_size) = 1.0 / static_cast<double>(p_size);
    }
    setWeights(dlib_vec);
}

bool ResponsiveModel::loadWeights(const std::string& filename) {
    WeightMaps p_maps, w_maps;
    bool ret = LoadBehaviorWeights(filename, p_maps, w_maps);
    if (ret) this->setWeights(p_maps, w_maps);
    return ret;
}

bool ResponsiveModel::saveWeights(const std::string& filename) {
    WeightMaps p_maps, w_maps;
    this->getWeights(p_maps, w_maps);
    bool ret = SaveBehaviorWeights(p_maps, w_maps, filename);
    return ret;
}

void ResponsiveModel::setTimeScale(float scale) {
    printf("* Set time scale (%f)\n", scale);
    time_scale = scale;
    for (int i = 0; i < p_behavior_coll.size(); i++) {
        p_behavior_coll[i]->setTimeScale(scale);
    }
    for (int i = 0; i < w_behavior_coll.size(); i++) {
        w_behavior_coll[i]->setTimeScale(scale);
    }
}

void ResponsiveModel::exitAllBehaviors() {
    for (int i = 0; i < p_behavior_coll.size(); i++) {
        p_behavior_coll[i]->exitBehavior();
    }
    for (int i = 0; i < w_behavior_coll.size(); i++) {
        w_behavior_coll[i]->exitBehavior();
    }
}

// ===== GL UIs =====
void ResponsiveModel::initGlUi() {
    if (inited) {
        modules.initGlUi();

        // Create and initialize plotters
        CreatePartPlotters(p_plotters, "Priority Movement ");
        CreatePartPlotters(w_plotters, "Weighted Movement ");
        InitPlottersGlUi(p_plotters);
        InitPlottersGlUi(w_plotters);
    }
}

void ResponsiveModel::drawGlUi() {
    if (inited) {
        // Modules window
        modules.drawGlUi();

        // Common Window
        ImGui::Begin("Models");
        // Model flags
        ImGui::Checkbox("Output Projection", &out_projection);
        ImGui::Checkbox("Plot Movements", &plot_movements);
        ImGui::Separator();

        // Weights io (load)
        ImGui::InputText("##model_load", load_filename, sizeof(load_filename));
        ImGui::SameLine();
        if (ImGui::Button("Load Weights")) {
            // Load and set result text
            if (loadWeights(std::string(load_filename))) {
                load_ret_str = ">> succeeded";
            } else {
                load_ret_str = ">> failed";
            }
        }
        ImGui::SameLine();
        ImGui::Text("%s", load_ret_str.c_str());

        // Weights io (save)
        ImGui::InputText("##model_save", save_filename, sizeof(save_filename));
        ImGui::SameLine();
        if (ImGui::Button("Save Weights")) {
            // Save and set result text
            if (saveWeights(std::string(save_filename))) {
                save_ret_str = ">> succeeded";
            } else {
                save_ret_str = ">> failed";
            }
        }
        ImGui::SameLine();
        ImGui::Text("%s", save_ret_str.c_str());

        // Set weight buttons
        if (ImGui::Button("Set base weights")) {
            setBaseWeights();
        }
        ImGui::Separator();

        // Behaviors
        p_behavior_coll.drawGlUi();
        w_behavior_coll.drawGlUi();

        // TimeScale
        ImGui::Separator();
        if (ImGui::DragFloat("Time scale", &time_scale, 0.1f, 0.f)) {
            setTimeScale(time_scale);
        }
        ImGui::End();

        // Plot priorities and weights (Independent windows)
        if (plot_movements) {
            // New window
            ImGui::Begin("Priority Movements", NULL,
                         ImGuiWindowFlags_AlwaysAutoResize);
            DrawPlottersGlUi(p_plotters);
            ImGui::End();
            // New window
            ImGui::Begin("Weighted Movements", NULL,
                         ImGuiWindowFlags_AlwaysAutoResize);
            DrawPlottersGlUi(w_plotters);
            ImGui::End();
        }
    }
}
