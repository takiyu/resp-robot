#include <stdio.h>
#include <fstream>
#include <sstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "../modules/camera/camera.h"

const std::string ARG_KEYS =
    "{help h usage ? |      | Print this message   }"
    "{@outdir        |<none>| Output directory path}";

bool FileExists(const std::string &filename) {
    FILE *fp = fopen(filename.c_str(), "r");
    if (fp != NULL) {
        fclose(fp);
        return true;
    } else {
        return false;
    }
}

int main(int argc, char const *argv[]) {
    // Arguments
    cv::CommandLineParser parser(argc, argv, ARG_KEYS);
    parser.about("Camera Capturing Tool");
    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }
    const std::string OUTDIR_PATH = parser.get<std::string>("@outdir");
    if (!parser.check()) {
        parser.printErrors();
        return 0;
    }

    // Open camera
    WebCamera webcam;
    webcam.open();

    int image_cnt = 0;
    printf("* Start main loop\n");
    printf(" > q: Quit\n");
    printf(" > s: Save capture\n");
    while (true) {
        // Capture
        cv::Mat frame = webcam.capture();
        if (frame.empty()) continue;

        // IO
        cv::imshow("frame", frame);
        char k = cv::waitKey(1);
        if (k == 'q') {
            break;
        } else if (k == 's') {
            // Save capture
            std::string img_path;
            // Create saving image path
            while (true) {
                image_cnt++;
                std::stringstream ss;
                ss << OUTDIR_PATH << "/" << image_cnt << ".png";
                img_path = ss.str();
                if (!FileExists(img_path)) {
                    break;
                }
            }
            printf("* Save current frame: %s\n", img_path.c_str());
            imwrite(img_path, frame);
        }
    }

    return 0;
}
