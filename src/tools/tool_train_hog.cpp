#include <opencv2/core/core.hpp>

#include <iostream>
#include <string>
#include <vector>

#include "../modules/camera/object_detector.h"

/*
 * note:
 *   Rotation of training data makes the accuracy lower (so disabled).
 * */

const std::string ARG_KEYS =
    "{help h usage ? |      | Print this message        }"
    "{@inxml         |<none>| Input xml path            }"
    "{@outhog        |<none>| Output serialized hog path}"
    "{window_size    |80    | Sliding window size       }"
    "{svm_c          |3     | SVM c parameter           }"
    "{gap_limit      |0.01  | Training gap limit        }"
    "{n_threads      |4     | Training thread number    }"
    // "{n_rotation     |3     | Rotated data number       }"
    ;

int main(int argc, char** argv) {
    // Arguments
    cv::CommandLineParser parser(argc, argv, ARG_KEYS);
    parser.about("Train HOG Tool");
    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }
    const std::string IN_XML_PATH = parser.get<std::string>("@inxml");
    const std::string OUT_HOG_PATH = parser.get<std::string>("@outhog");
    const int WINDOW_SIZE = parser.get<int>("window_size");
    const double SVM_C = parser.get<double>("svm_c");
    const double GAP_LIMIT = parser.get<double>("gap_limit");
    const int N_THREADS = parser.get<int>("n_threads");
    // const int N_ROTATION = parser.get<int>("n_rotation");
    if (!parser.check()) {
        parser.printErrors();
        return 0;
    }

    // Print arguments
    printf("** HOG-SVM Training Tool **\n");
    printf(" > window_size: %d\n", WINDOW_SIZE);
    printf(" > svm_c: %f\n", SVM_C);
    printf(" > gap_limit: %f\n", GAP_LIMIT);
    printf(" > n_threads: %d\n", N_THREADS);
    // printf(" > n_rotation: %d\n", N_ROTATION);

    // Train
    HogDetector hog_detector;
    hog_detector.train(IN_XML_PATH, WINDOW_SIZE, SVM_C, GAP_LIMIT, N_THREADS);

    // Save
    hog_detector.save(OUT_HOG_PATH);
}
