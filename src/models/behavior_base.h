#ifndef BEHAVIOR_BASE_160914
#define BEHAVIOR_BASE_160914
#include <boost/thread.hpp>
#include <map>
#include <string>
#include <vector>

#include "containers.h"

// =============================== Base Behavior ===============================
class BaseBehavior {
public:
    BaseBehavior(const std::string& tag,
                 bool auto_occurring)
        : TAG(tag),
          auto_occurring(auto_occurring),
          behavior_thread(NULL),
          running(false),
          time_scale(1.f) {}
    ~BaseBehavior() { exitBehavior(); }

    const std::string TAG;

    // Basic methods
    void init();
    void setSensor(const SensorCtr& sensor_ctr);
    void getMovement(MovementCtr& behavior_ctr);

    // Set only declared weight if declaration flag is false
    void setWeight(const std::string& label, float w, bool declaration = false);
    // Get only declared weight
    float getWeight(const std::string& label) const;
    // Get all of the weight map
    std::map<std::string, float> getWeightMap() { return weight_map; }  // deep

    // Set time scale (2 -> two times speed)
    void setTimeScale(float scale) { time_scale = scale; }

    // Start and exit a thread
    void startBehavior();
    void exitBehavior();

    // GL UI
    void drawGlUi(bool auto_occurring_btn = true);

    void setAutoOccurring(bool v) { auto_occurring = v; }

protected:
    // status buffers
    ThreadCopiablePtr<SensorCtr> sensor_ctr;      // in
    ThreadCopiablePtr<MovementCtr> behavior_ctr;  // out

    // Implement declaration of weights here.
    virtual void declareWeights() = 0;
    // Implement behavior occurring condition here.
    virtual bool shouldOccur() = 0;
    // Implement blocking behavior here. (called in thread)
    virtual void behave() = 0;

    // Overwrite common variables
    float overwriteMs(float ms) {
        return ms / time_scale;
    } 
    float overwriteSpeed(float max_speed) {
        return max_speed * time_scale;
    }
    float overwriteAccel(float accel) {
        return accel * time_scale * time_scale;
    }
    bool auto_occurring;

private:
    bool running;  // flag for thread running
    boost::thread* behavior_thread;
    std::map<std::string, float> weight_map;
    float time_scale;
};

// ============================ Behavior Collection ============================
class BehaviorCollection {
public:
    BehaviorCollection(const std::string& tag) : TAG(tag), n_behaviors(0) {}
    ~BehaviorCollection() {}

    const std::string TAG;

    void add(BaseBehavior* behavior);
    void initAll();
    void releaseAll();

    int size() const { return n_behaviors; }
    std::string tag(int i) const { return behaviors[i]->TAG; }

    // Raw access
    BaseBehavior* operator[](const int i) { return behaviors[i]; }
    BaseBehavior* operator[](const int i) const { return behaviors[i]; }

    // GL UI
    void drawGlUi();

private:
    int n_behaviors;
    std::vector<BaseBehavior*> behaviors;
};

#endif
