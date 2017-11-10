#ifndef RESOPONSIVE_MODEL_H_160914
#define RESOPONSIVE_MODEL_H_160914

#include <string>
#include <vector>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>

#include "../modules/modules.h"
#include "../viewer/plotter.h"
#include "behavior_base.h"
#include "behavior_weights.h"
#include "behaviors.h"
#include "containers.h"
#include "mergers.h"
#include "projector.h"
#include "sensors.h"

class ResponsiveModel {
public:
    ResponsiveModel()
        : inited(false),
          out_projection(true),
          plot_movements(false),
          sensors(&modules),
          projector(&modules),
          p_behavior_coll("Priority Behaviors"),
          w_behavior_coll("Weighted Behaviors"),
          do_overwrite_sensor(false),
          time_scale(1.f) {}
    ~ResponsiveModel() {
        exit();
    }

    // Basic methods
    void init();
    void exit();
    void update(bool set_only = false);

    // Weights
    void setWeights(const WeightMaps& p_maps, const WeightMaps& w_maps);
    void getWeights(WeightMaps& p_maps, WeightMaps& w_maps);
    void setWeights(const DlibColVec& dlib_vec);
    void getWeights(DlibColVec& dlib_vec);
    void setBaseWeights();
    bool loadWeights(const std::string& filename);
    bool saveWeights(const std::string& filename);

    // Time scale
    void setTimeScale(float scale);

    // Optimization methods
    void setOverwriteSensor(const SensorCtr& sensor_ctr) {
        do_overwrite_sensor = true;
        overwrite_sensor = sensor_ctr;
    }
    void clearOverwriteSensor() { do_overwrite_sensor = false; }
    void getResultBehavior(MovementCtr &result_behavior) {
        result_behavior = cache_result_behavior;
    }

    void exitAllBehaviors();

    // GL UIs
    void initGlUi();
    void drawGlUi();

private:
    bool inited, out_projection, plot_movements;

    Modules modules;
    Sensors sensors;
    BehaviorCollection p_behavior_coll, w_behavior_coll;
    RobovieProjector projector;
    BehaviorWeightsConverter weights_converter;

    // Optimization variables
    bool do_overwrite_sensor;
    SensorCtr overwrite_sensor;
    MovementCtr cache_result_behavior;

    // Plotters
    std::vector<TimeSeriesPlotter> w_plotters, p_plotters;

    // GL UI
    char load_filename[50] = "../data/weights.json";
    char save_filename[50] = "../data/out_weights.json";
    std::string save_ret_str, load_ret_str;
    float time_scale;
};

#endif
