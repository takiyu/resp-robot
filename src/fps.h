#ifndef FPSCOUNTER_H_141021
#define FPSCOUNTER_H_141021

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

class FpsCounter {
public:
    FpsCounter();
    void init();
    int update();
    int getFps() const { return fps; }

private:
    int frame_count;
    int pre_frame_count;
    long now_time;
    long time_diff;
    int fps;
    double frequency;
    long start_time;
};

#endif
