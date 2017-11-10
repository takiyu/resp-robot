#ifndef SENSORS_H_160913
#define SENSORS_H_160913

#include "../modules/modules.h"
#include "containers.h"

// ================================== Sensors ==================================
class Sensors {
public:
    Sensors(Modules *modules) { this->modules = modules; }
    ~Sensors() {}
    void get(SensorCtr &sensor_ctr);

private:
    Modules *modules;
};

#endif
