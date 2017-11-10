#include <iostream>
#include <string>
#include <vector>

#include "../modules/camera/object_detector.h"

const std::string ARG_KEYS =
    "{help h usage ? |      | Print this message                        }"
    "{@inxmls        |<none>| Input xml paths split by ';'              }"
    "{@outsvm        |<none>| Output serialized svm path                }"
    "{svm_c          |1     | SVM c parameter                           }"
    "{neg_per_img    |3     | The number of Negative sample per a image }";

std::vector<std::string> split(const std::string &str, char sep) {
    std::vector<std::string> res;
    std::stringstream ss(str);
    std::string buf;
    while (std::getline(ss, buf, sep)) {
        if (!buf.empty()) {
            res.push_back(buf);
        }
    }
    return res;
}

int main(int argc, char **argv) {
    // Arguments
    cv::CommandLineParser parser(argc, argv, ARG_KEYS);
    parser.about("Train classification SVM Tool");
    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }
    const std::string IN_XML_PATHS_RAW = parser.get<std::string>("@inxmls");
    const std::string OUT_SVM_PATH = parser.get<std::string>("@outsvm");
    const double SVM_C = parser.get<double>("svm_c");
    const int NEG_PER_IMG = parser.get<double>("neg_per_img");
    if (!parser.check()) {
        parser.printErrors();
        return 0;
    }
    const std::vector<std::string> IN_XML_PATHS = split(IN_XML_PATHS_RAW, ';');

    // Print arguments
    printf("** SVM Training Tool on Raw Images\n");
    printf(" > svm_c: %f\n", SVM_C);
    printf(" > neg_per_img: %d\n", NEG_PER_IMG);

    // Train
    SvmChipClassifier classifier;
    classifier.train(IN_XML_PATHS, SVM_C, NEG_PER_IMG);

    // Save
    classifier.save(OUT_SVM_PATH);
}
