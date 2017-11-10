#include <stdio.h>
#include <fstream>
#include <sstream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "../fps.h"
#include "../modules/camera/camera.h"
#include "../modules/camera/object_detector.h"

int main(int argc, char const* argv[]) {
    // Open camera
    WebCamera webcam;
    webcam.open();

    // Object detector
    ObjectDetector detector;
    detector.init();

    // Fps
    FpsCounter fps;

    printf("* Start tracking\n");
    while (true) {
        // Capture
        cv::Mat frame = webcam.capture();
        if (frame.empty()) continue;

        // Detect and draw rectangles
        std::vector<std::vector<cv::Rect> > object_bboxes;
        std::vector<std::vector<cv::Point3f> > object_positions;
        detector.detect(frame, object_bboxes, object_positions);
        detector.drawDebug(frame, object_bboxes);

        // Print 3D coordinates
        for (int label = 0; label < object_bboxes.size(); label++) {
            for (int i = 0; i < object_bboxes[label].size(); i++) {
                std::cout << "obj " << label
                          << " pos:" << object_positions[label][i] << std::endl;
            }
        }

        // IO
        std::cout << fps.update() << std::endl;
        cv::imshow("frame", frame);
        char k = cv::waitKey(1);
        if (k == 'q') {
            break;
        }
    }

    return 0;
}
