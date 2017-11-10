#ifndef WEIGHTS_OPTIMIZER_H_161218
#define WEIGHTS_OPTIMIZER_H_161218

#include "resp_model.h"

// Weights optimization with scenario json
class BehaviorWeightsOptimizer {
public:
    BehaviorWeightsOptimizer()
        : v_coeff(0.001f),
          p_coeff(1.f),
          optimized(false) {}
    ~BehaviorWeightsOptimizer() {}

    void init();
    void exit();

    bool loadScenarios(const std::string& filename);
    void clearScenarios() { scenarios.clear(); }

    // Main optimization
    double optimize();
    double evaluateOne(const DlibColVec& dlib_vec, bool print_eval) const;
    // Evaluation operator
    double operator()(const DlibColVec& dlib_vec) const {
        return evaluateOne(dlib_vec, false); // no evaluation print
    }

    // GL UIs
    void initGlUi();
    void drawGlUi();

    // Scenario element
    struct ScenarioElem {
        SensorCtr sensor;
        MovementEvaluationCtr behavior;
        float period_ms;
        float importance;
    };

private:
    // Scenario
    std::map<std::string, std::vector<ScenarioElem> > scenarios;
    // Main model (mutable because of const operator())
    mutable ResponsiveModel resp_model;

    // Coefficients
    float v_coeff, p_coeff;

    // GL UI (scenario)
    char load_filename[50] = "../data/scenario0.json";
    std::string scenario_ret_str;
    // GL UI (optimization)
    bool optimized;
    double latest_error;

    const float TIME_SCALE = 10.f, WAIT_RESP_MS = 20.f;
};

#endif
