#include "behavior_base.h"

#include "imgui/imgui.h"

// ============================== Base  Behavior ===============================
void BaseBehavior::init() {
    weight_map.clear();
    declareWeights();
}

void BaseBehavior::setSensor(const SensorCtr& sensor_ctr) {
    // Inputs (copy)
    this->sensor_ctr.set(sensor_ctr);

    // Behavior occurs automatically
    if (auto_occurring) {
        if (!running && shouldOccur()) {
            startBehavior();
        }
    }
}

void BaseBehavior::getMovement(MovementCtr& behavior_ctr) {
    // Output (copy and set used flags)
    if (running) {
        this->behavior_ctr.lock();
        behavior_ctr = this->behavior_ctr->use();
        this->behavior_ctr.unlock();
    }

    // Set tag name
    behavior_ctr.tag = TAG;
    // Set merged history (first value)
    behavior_ctr.body.merged_history[TAG] = 1.f;
    behavior_ctr.head.merged_history[TAG] = 1.f;
    behavior_ctr.eyes.merged_history[TAG] = 1.f;
    behavior_ctr.eyelid.merged_history[TAG] = 1.f;
    behavior_ctr.right_arm.merged_history[TAG] = 1.f;
    behavior_ctr.left_arm.merged_history[TAG] = 1.f;
    behavior_ctr.voice.merged_history[TAG] = 1.f;
}

void BaseBehavior::setWeight(const std::string& label, float w,
                             bool declaration) {
    // Check declaration and its flag
    if (weight_map.count(label) == 0 && !declaration) {
        printf(" >> [%s] Invalid weight access to set (%s)\n", TAG.c_str(),
               label.c_str());
        return;
    }
    // Set
    weight_map[label] = w;
}

float BaseBehavior::getWeight(const std::string& label) const {
    // Check declaration
    if (weight_map.count(label) == 0) {
        printf(" >> [%s] Invalid weight access to get (%s)\n", TAG.c_str(),
               label.c_str());
        return 0.f;  // empty value
    }
    // Get
    return weight_map.at(label);
}

void BaseBehavior::drawGlUi(bool auto_occurring_btn) {
    // Common window
    ImGui::Begin("Models");
    // Text
    ImGui::Text("%-23s", TAG.c_str());
    ImGui::SameLine();
#if 1
    // Toggle start and stop
    if (ImGui::Button(("Toggle start/exit##" + TAG).c_str())) {
        if (running) {
            exitBehavior();
        } else {
            startBehavior();
        }
    }
#else
    // Force start button
    if (ImGui::Button(("(Start forcibly)##" + TAG).c_str())) {
        startBehavior();
    }
    ImGui::SameLine();
    // Force exit button
    if (ImGui::Button(("(Exit forcibly)##" + TAG).c_str())) {
        exitBehavior();
    }
    ImGui::SameLine();
#endif
    if (auto_occurring_btn) {
        // Auto occurring
        ImGui::SameLine();
        ImGui::Checkbox(("Auto occuring##" + TAG).c_str(), &auto_occurring);
    }
    // Running text
    if (running) {
        ImGui::SameLine();
        ImGui::Text("(running)");
    }
    ImGui::End();
}

void BaseBehavior::startBehavior() {
    if (!running) {
        // Delete previous thread
        exitBehavior();
        // Start new thread
        // printf("* [%s] Start a behavior thread\n", TAG.c_str());
        behavior_ctr.reset();  // clean
        running = true;
        behavior_thread = new boost::thread([&]() {
            behave();  // main behavior
            // printf("* [%s] Exit a behavior thread (auto)\n", TAG.c_str());

            // Wait for update of main thread
            while (true) {
                behavior_ctr.lock();
                bool empty_or_used = behavior_ctr->isAllEmptyOrUsed();
                behavior_ctr.unlock();
                if (empty_or_used) break;
                boost::this_thread::sleep(boost::posix_time::milliseconds(5));
            }
            running = false;
        });
    }
}

void BaseBehavior::exitBehavior() {
    if (behavior_thread != NULL) {
        if (running) {
            printf("* [%s] Exit a behavior thread (force)\n", TAG.c_str());
            behavior_thread->interrupt();
            behavior_thread->join();
        }
        delete behavior_thread;
        behavior_thread = NULL;
        running = false;
    }
}

// ============================ Behavior Collection ============================
void BehaviorCollection::add(BaseBehavior* behavior) {
    // Register new behavior
    printf("* Add new behavior: %s\n", behavior->TAG.c_str());
    behaviors.push_back(behavior);
    n_behaviors = behaviors.size();
}

void BehaviorCollection::initAll() {
    if (behaviors.size() > 0) {
        printf("* Initialize all behaviors\n");
        for (int i = 0; i < behaviors.size(); i++) {
            behaviors[i]->init();
        }
    }
}

void BehaviorCollection::releaseAll() {
    if (behaviors.size() > 0) {
        printf("* Release all behaviors\n");
        // Delete behaviors
        for (int i = 0; i < behaviors.size(); i++) {
            delete behaviors[i];
        }
        behaviors.clear();
        n_behaviors = 0;
    }
}

void BehaviorCollection::drawGlUi() {
    // Common window
    ImGui::Begin("Models");
    // Header
    if (ImGui::CollapsingHeader(TAG.c_str(), ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::Button(("Toggle Auto Occurring##" + TAG).c_str())) {
            static bool v = true;
            v = !v;
            for (int i = 0; i < n_behaviors; i++) {
                behaviors[i]->setAutoOccurring(v);
            }
        }
        static bool auto_occurring_btn = true;
        ImGui::SameLine();
        if (ImGui::Button(("Show/Hide Auto Occurring##" + TAG).c_str())) {
            auto_occurring_btn = !auto_occurring_btn;
        }
        // Draw each behavior
        for (int i = 0; i < n_behaviors; i++) {
            behaviors[i]->drawGlUi(auto_occurring_btn);
        }
    }
    ImGui::End();
}
