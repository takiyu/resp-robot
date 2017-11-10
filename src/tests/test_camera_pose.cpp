#ifndef NO_TORCH  // build without Torch
#include <stdio.h>
#include <fstream>
#include <sstream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "../fps.h"
#include "../modules/camera/camera.h"
#include "../modules/camera/pose_estimator.h"

int main(int argc, char const* argv[]) {
    // Open camera
    WebCamera webcam;
    webcam.open();

    PoseEstimator pose_estimator;

    while (true) {
        cv::Mat frame = webcam.capture();
        if (frame.empty()) continue;

        pose_estimator.estimate(frame);
        pose_estimator.drawDebug(frame);

        cv::imshow("frame", frame);
        char k = cv::waitKey(1);
        if (k == 'q') {
            break;
        }
    }

    return 0;
}

#else

#include <stdio.h>
int main(int argc, char const* argv[]) {
    printf("Disabled with NO_TORCH flag.");
    return 0;
}

#endif
