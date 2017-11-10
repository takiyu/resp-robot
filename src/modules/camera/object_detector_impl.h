#ifndef OBJECT_DETECTOR_IMPL_160727
#define OBJECT_DETECTOR_IMPL_160727
#include "object_detector.h"

#include <dlib/image_processing.h>

#include <opencv2/core/core.hpp>
#include <string>

// HOG type that uses downsampled images at a ratio of 5/6
typedef dlib::scan_fhog_pyramid<dlib::pyramid_down<6> > hog_scanner_type;
// Classifier SVM types
typedef dlib::matrix<double> svm_sample_type;
typedef dlib::linear_kernel<svm_sample_type> svm_kernel_type;
typedef dlib::multiclass_linear_decision_function<svm_kernel_type, int>
    svm_dec_funct_type;
typedef dlib::normalized_function<svm_dec_funct_type> svm_funct_type;

class HogDetector::Impl {
public:
    Impl() : empty(true) {}
    ~Impl() {}

    void load(const std::string& filename);
    void save(const std::string& filename);
    void detect(const dlib::array2d<dlib::bgr_pixel>& dlib_img,
                std::vector<dlib::rect_detection>& dets);
    void detect(const cv::Mat& src, std::vector<cv::Rect>& bboxes);
    void train(const std::string& inxml_path, int window_size, double svm_c,
               double gap_limit, int n_threads);

private:
    bool empty;
    dlib::object_detector<hog_scanner_type> detector;
};

class MultiHogDetector::Impl {
public:
    Impl() {}
    ~Impl() {}

    void clear() { detectors.clear(); }
    void loadNext(const std::string& filename);
    void detect(const dlib::array2d<dlib::bgr_pixel>& dlib_img,
                std::vector<dlib::rect_detection>& dets);
    void detect(const cv::Mat& src, std::vector<cv::Rect>& bboxes);

private:
    std::vector<dlib::object_detector<hog_scanner_type> > detectors;
};

class SvmChipClassifier::Impl {
public:
    Impl() : empty(true), CHIP_SIZE(20) {}
    ~Impl() {}

    void load(const std::string& filename);
    void save(const std::string& filename);
    int classify(const dlib::array2d<dlib::bgr_pixel>& dlib_img,
                 const dlib::rectangle& drect);
    int classify(const cv::Mat& src, const cv::Rect& rect);
    void train(const std::vector<std::string>& inxml_paths, double svm_c,
               int neg_per_img);

private:
    const int CHIP_SIZE;
    bool empty;
    svm_funct_type classifier;
};

class ObjectDetector::Impl {
public:
    Impl() : empty(true), DO_SINGULATE(true), HOG_INTERVAL(4) {}
    ~Impl() {}

    void init();
    void detect(const cv::Mat& src,
                std::vector<std::vector<cv::Rect> >& bboxes);
    void detect(const cv::Mat& src, std::vector<std::vector<cv::Rect> >& bboxes,
                std::vector<std::vector<cv::Point3f> >& positions);

    void drawDebug(cv::Mat& img,
                   const std::vector<std::vector<cv::Rect> >& bboxes);

private:
    bool empty;
    const bool DO_SINGULATE;
    const int HOG_INTERVAL;
    int interval_cnt;

    MultiHogDetector hog_detector;
    SvmChipClassifier classifier;

    // Tracker
    cv::Mat prev_frame;
    std::vector<std::vector<cv::Rect> > prev_bboxes;
    std::vector<char> tracking_states;
    std::vector<dlib::correlation_tracker> trackers;

    // Template of 3D object rectangles
    std::vector<std::vector<cv::Point3f> > objects_3d_points;
};

#endif
