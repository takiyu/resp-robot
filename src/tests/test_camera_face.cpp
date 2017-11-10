#include <stdio.h>
#include <fstream>
#include <sstream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "../fps.h"
#include "../modules//camera/camera.h"
#include "../modules/camera/face_estimator.h"

int main(int argc, char const* argv[]) {
    // Open camera
    WebCamera webcam;
    webcam.open();

    // Face Estimator
    FaceEstimator face_estimator;
    face_estimator.init();

    // Fps
    FpsCounter fps;

    printf("* Start tracking");
    while (true) {
        // Capture
        cv::Mat frame = webcam.capture();
        if (frame.empty()) continue;
        cv::Mat gray;
        cv::cvtColor(frame, gray, CV_BGR2GRAY);

        // Estimate and draw
        face_estimator.estimate(gray);
        face_estimator.drawDebug(frame);

        // IO
        std::cout << fps.update() << std::endl;
        cv::imshow("frame", frame);
        char k = cv::waitKey(1);
        if (k == 'r') {
            face_estimator.reset();
        } else if (k == 'q') {
            break;
        }
    }

    return 0;
}
