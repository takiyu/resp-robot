#ifndef OBJECT_DETECTOR_160712
#define OBJECT_DETECTOR_160712

#include <opencv2/core/core.hpp>
#include <string>

#include "../../config.h"

class HogDetector {
public:
    HogDetector();
    ~HogDetector();

    void load(const std::string& filename);
    void save(const std::string& filename);
    void detect(const cv::Mat& src, std::vector<cv::Rect>& bboxes);
    void train(const std::string& inxml_path, int window_size, double svm_c,
               double gap_limit, int n_threads);

    // Reference to implementation class.
    // To use internal methods, please include `object_detector_impl.h`
    class Impl;
    Impl* impl;
};

class MultiHogDetector {
public:
    MultiHogDetector();
    ~MultiHogDetector();

    void clear();
    void loadNext(const std::string& filename);
    void detect(const cv::Mat& src, std::vector<cv::Rect>& bboxes);

    class Impl;
    Impl* impl;
};

class SvmChipClassifier {
public:
    SvmChipClassifier();
    ~SvmChipClassifier();

    void load(const std::string& filename);
    void save(const std::string& filename);
    int classify(const cv::Mat& src, const cv::Rect& rect);
    void train(const std::vector<std::string>& inxml_paths, double svm_c,
               int neg_par_img);

    class Impl;
    Impl* impl;
};

class ObjectDetector {
public:
    ObjectDetector();
    ~ObjectDetector();

    void init();
    void detect(const cv::Mat& src,
                std::vector<std::vector<cv::Rect> >& bboxes);
    void detect(const cv::Mat& src, std::vector<std::vector<cv::Rect> >& bboxes,
                std::vector<std::vector<cv::Point3f> >& positions);

    void drawDebug(cv::Mat& img,
                   const std::vector<std::vector<cv::Rect> >& bboxes);

    class Impl;
    Impl* impl;
};

#endif
