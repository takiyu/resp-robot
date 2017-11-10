#ifndef PROJECTOR_H_160914
#define PROJECTOR_H_160914

#include "containers.h"

class RobovieProjector {
public:
    RobovieProjector(Modules *modules, float max_speed = 8.f, float accel = 1.f)
        : MAX_SPEED(max_speed), ACCEL(accel) {
        this->modules = modules;
    }
    ~RobovieProjector() {}
    void project(const MovementCtr &result_behavior);

private:
    const float MAX_SPEED, ACCEL;
    Modules *modules;
};

#endif
