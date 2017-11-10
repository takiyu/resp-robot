#include "object_detector.h"
#include "object_detector_impl.h"

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <dlib/data_io.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_processing.h>
#include <dlib/opencv.h>
#include <dlib/svm_threaded.h>

#include <sstream>
#include <string>

#include "camera.h"

namespace {

// Convert OpenCV Mat to dlib array
template <typename T>
void ConvertMat(const cv::Mat& src, dlib::array2d<T>& dst) {
    dlib::cv_image<T> dlib_cimg(src);
    dlib::assign_image(dst, dlib_cimg);
}

// Convert OpenCV Rect to dlib rectangle
dlib::rectangle ConvertRectangle(const cv::Rect& src) {
    return dlib::rectangle(src.x, src.y, src.x + src.width - 1,
                           src.y + src.height - 1);
}

// Convert dlib rectangle to OpenCV Rect
cv::Rect ConvertRectangle(const dlib::rectangle& src) {
    return cv::Rect(src.left(), src.top(), src.width(), src.height());
}

// Convert dlib rect_detection to OpenCV Rect
void ConvertRectangles(const std::vector<dlib::rect_detection>& src,
                       std::vector<cv::Rect>& dst) {
    int n = src.size();
    dst.resize(n);
    for (int i = 0; i < n; i++) {
        const dlib::rectangle& drect = src[i].rect;
        cv::Rect rect = ConvertRectangle(drect);
        dst[i] = rect;
    }
}

// Convert dlib image to classifier matrix
template <typename image_type1, typename chip_details>
void ConvertToSvmSample(const image_type1& in_img, const chip_details& location,
                        svm_sample_type& out_sample, const int img_size) {
    // roi
    image_type1 chip;
    dlib::extract_image_chip(in_img, location, chip);
    // resize
    dlib::matrix<dlib::bgr_pixel> resized_img;
    resized_img.set_size(img_size, img_size);
    dlib::resize_image(chip, resized_img, dlib::interpolate_bilinear());

    // uchar bgr 2dim image -> double single column matrix
    out_sample.set_size(img_size * img_size * 3, 1);
    for (int y = 0; y < img_size; y++) {
        for (int x = 0; x < img_size; x++) {
            out_sample((y * img_size + x) * 3 + 0, 0) = resized_img(y, x).blue;
            out_sample((y * img_size + x) * 3 + 1, 0) = resized_img(y, x).green;
            out_sample((y * img_size + x) * 3 + 2, 0) = resized_img(y, x).red;
        }
    }
}

void CreateNegativeSamples(const dlib::array2d<dlib::bgr_pixel>& img,
                           const std::vector<dlib::rectangle>& positive_bboxes,
                           std::vector<dlib::rectangle>& negative_rects,
                           int neg_per_img) {
    negative_rects.clear();

    // Selective search  TODO: Configure parameters
    std::vector<dlib::rectangle> ssrects;
    dlib::find_candidate_object_locations(img, ssrects,
                                          dlib::linspace(50, 200, 2), 2200);

    // Mix the order
    dlib::randomize_samples(ssrects);

    // Check overlapping with positive boxes
    int neg_cnt = 0;
    for (int ssrect_idx = 0; ssrect_idx < ssrects.size(); ssrect_idx++) {
        if (neg_cnt >= neg_per_img) break;
        // Check the overlapping
        bool intersected = false;
        for (int i = 0; i < positive_bboxes.size(); i++) {
            if (ssrects[ssrect_idx].intersect(positive_bboxes[i]).area() > 0) {
                intersected = true;
                break;
            }
        }
        if (!intersected) {
            // Register
            negative_rects.push_back(ssrects[ssrect_idx]);
            neg_cnt++;
        }
    }

    //     // [DEBUG] Show selective search rectangles
    //     dlib::image_window win;
    //     win.set_image(img);
    //     win.clear_overlay();
    //     win.add_overlay(ssrects);
    //     getchar();
}

cv::Rect ExpandRoiRect(const cv::Rect& rect, const cv::Mat& img, float scale) {
    float w = rect.width * scale;
    float h = rect.height * scale;
    cv::Rect roi_rect(rect.x - (w - rect.width) * 0.5f,
                      rect.y - (h - rect.height) * 0.5f, w, h);
    roi_rect &= cv::Rect(0, 0, img.cols, img.rows);
    return roi_rect;
}

}  // namespace

// ================================ HogDetector ================================
void HogDetector::Impl::load(const std::string& filename) {
    printf("* Load HogDetector file: %s\n", filename.c_str());
    dlib::deserialize(filename) >> detector;
    empty = false;
}

void HogDetector::Impl::save(const std::string& filename) {
    printf("* Save HogDetector file: %s\n", filename.c_str());
    if (empty) {
        printf(" >> empty\n");
        return;
    }
    dlib::serialize(filename) << detector;
}

void HogDetector::Impl::detect(const dlib::array2d<dlib::bgr_pixel>& dlib_img,
                               std::vector<dlib::rect_detection>& dets) {
    if (empty) {
        printf("* HogDetector is empty\n");
        dets.clear();
        return;
    }
    // Hog Detect
    detector(dlib_img, dets);
}

void HogDetector::Impl::detect(const cv::Mat& src,
                               std::vector<cv::Rect>& bboxes) {
    // Convert OpenCV Mat to dlib
    dlib::array2d<dlib::bgr_pixel> dlib_img;
    ConvertMat(src, dlib_img);
    // Detect
    std::vector<dlib::rect_detection> dets;
    detect(dlib_img, dets);
    // Parse dlib result to OpenCV
    ConvertRectangles(dets, bboxes);
}

void HogDetector::Impl::train(const std::string& inxml_path, int window_size,
                              double svm_c, double gap_limit, int n_threads) {
    printf("* Train HogDetector\n");
    // Load training data
    printf(" >> Load training data: %s\n", inxml_path.c_str());
    dlib::array<dlib::array2d<dlib::bgr_pixel> > imgs_train;
    std::vector<std::vector<dlib::rectangle> > boxes_train;
    dlib::load_image_dataset(imgs_train, boxes_train, inxml_path);
    printf("  > %lu images\n", imgs_train.size());

    printf(" >> Increase training data\n");
    // Double the size of the images to detect smaller objects
    dlib::upsample_image_dataset<dlib::pyramid_down<2> >(imgs_train,
                                                         boxes_train);
    // Increase the number of training dataset by flipping
    dlib::add_image_left_right_flips(imgs_train, boxes_train);
    // Increase the number of training dataset by rotation
    // dlib::add_image_rotations(dlib::linspace(0, dlib::pi, N_ROTATION),
    //                           imgs_train, boxes_train);
    printf("  > %lu images\n", imgs_train.size());

    // Create HOG scanner
    hog_scanner_type scanner;
    // Size of the sliding window detector
    scanner.set_detection_window_size(window_size, window_size);
    // SVM regularization parameter which helps with generalization
    // If it is too large, the SVM will not fit the data well
    // scanner.set_nuclear_norm_regularization_strength(1.0);

    // Create HOG Trainer
    dlib::structural_object_detection_trainer<hog_scanner_type> trainer(
        scanner);
    // The number of threads
    trainer.set_num_threads(n_threads);
    // SVM C parameter (Too larger parameter causes overfitting)
    trainer.set_c(svm_c);
    // Set to print training progress
    trainer.be_verbose();
    // Risk gap limitation
    trainer.set_epsilon(gap_limit);

    // Remove invalid data
    printf(" >> Remove invalid data\n");
    dlib::remove_unobtainable_rectangles(trainer, imgs_train, boxes_train);
    printf("  > %lu images\n", imgs_train.size());

    printf(" >> Start to train\n");
    // Start to train
    detector = trainer.train(imgs_train, boxes_train);

    printf(" >> Evaluation\n");
    // Evaluate
    dlib::matrix<double, 1, 3> test_ret =
        dlib::test_object_detection_function(detector, imgs_train, boxes_train);
    std::cout << "  > Training results: " << test_ret << std::endl;
    // Show trained HOG
    dlib::image_window hog_win(draw_fhog(detector), "Learned fHOG detector");
    std::cout << "Input some key to console" << std::endl;
    std::cin.get();

    empty = false;
}

// ===== exposed methods =====
HogDetector::HogDetector() { impl = new Impl; }
HogDetector::~HogDetector() { delete impl; }
void HogDetector::load(const std::string& filename) { impl->load(filename); }
void HogDetector::save(const std::string& filename) { impl->save(filename); }
void HogDetector::detect(const cv::Mat& src, std::vector<cv::Rect>& bboxes) {
    impl->detect(src, bboxes);
}
void HogDetector::train(const std::string& inxml_path, int window_size,
                        double svm_c, double gap_limit, int n_threads) {
    impl->train(inxml_path, window_size, svm_c, gap_limit, n_threads);
}

// ============================= HogMultiDetector ==============================
void MultiHogDetector::Impl::loadNext(const std::string& filename) {
    printf("* Load New HogDetector file: %s\n", filename.c_str());
    // Load and append
    dlib::object_detector<hog_scanner_type> detector;
    dlib::deserialize(filename) >> detector;
    detectors.push_back(detector);
}

void MultiHogDetector::Impl::detect(
    const dlib::array2d<dlib::bgr_pixel>& dlib_img,
    std::vector<dlib::rect_detection>& dets) {
    if (detectors.size() == 0) {
        printf("* MultiHogDetector is empty\n");
        dets.clear();
        return;
    }
    // Hog Detect
    dlib::evaluate_detectors(detectors, dlib_img, dets);
}
void MultiHogDetector::Impl::detect(const cv::Mat& src,
                                    std::vector<cv::Rect>& bboxes) {
    // Convert OpenCV Mat to dlib
    dlib::array2d<dlib::bgr_pixel> dlib_img;
    ConvertMat(src, dlib_img);
    // Detect
    std::vector<dlib::rect_detection> dets;
    detect(dlib_img, dets);
    // Parse dlib result to OpenCV
    ConvertRectangles(dets, bboxes);
}

// ===== exposed methods =====
MultiHogDetector::MultiHogDetector() { impl = new Impl; }
MultiHogDetector::~MultiHogDetector() { delete impl; }
void MultiHogDetector::clear() { impl->clear(); }
void MultiHogDetector::loadNext(const std::string& filename) {
    impl->loadNext(filename);
}
void MultiHogDetector::detect(const cv::Mat& src,
                              std::vector<cv::Rect>& bboxes) {
    impl->detect(src, bboxes);
}

// ============================ SvmImageClassifier =============================
void SvmChipClassifier::Impl::load(const std::string& filename) {
    printf("* Load SvmChipClassifier file: %s\n", filename.c_str());
    dlib::deserialize(filename) >> classifier;
    empty = false;
}

void SvmChipClassifier::Impl::save(const std::string& filename) {
    printf("* Save SvmChipClassifier file: %s\n", filename.c_str());
    if (empty) {
        printf(" >> empty\n");
        return;
    }
    dlib::serialize(filename) << classifier;
}

int SvmChipClassifier::Impl::classify(
    const dlib::array2d<dlib::bgr_pixel>& dlib_img,
    const dlib::rectangle& drect) {
    if (empty) {
        printf("* SvmChipClassifier is empty\n");
        return -1;
    }
    svm_sample_type sample;
    ConvertToSvmSample(dlib_img, drect, sample, CHIP_SIZE);
    return classifier.function(classifier.normalizer(sample));
}

int SvmChipClassifier::Impl::classify(const cv::Mat& src,
                                      const cv::Rect& rect) {
    // Convert OpenCV Mat to dlib
    dlib::array2d<dlib::bgr_pixel> dlib_img;
    ConvertMat(src, dlib_img);
    // Convert OpenCV Rect to dlib
    dlib::rectangle drect = ConvertRectangle(rect);
    // Classify
    return classify(dlib_img, drect);
}

void SvmChipClassifier::Impl::train(const std::vector<std::string>& inxml_paths,
                                    double svm_c, int neg_per_img) {
    printf("* Train SvmChipClassifier\n");
    std::vector<svm_sample_type> samples;
    std::vector<int> labels;
    for (int i = 0; i < inxml_paths.size(); i++) {
        // Load xml
        printf(" >> Load training data: %s\n", inxml_paths[i].c_str());
        dlib::array<dlib::array2d<dlib::bgr_pixel> > imgs_train;
        std::vector<std::vector<dlib::rectangle> > boxes_train;
        dlib::load_image_dataset(imgs_train, boxes_train, inxml_paths[i]);
        printf("  > images       : %lu\n", imgs_train.size());

        // Increase the number of training dataset by flipping
        dlib::add_image_left_right_flips(imgs_train, boxes_train);
        printf("  > bboxes (pos) : %lu\n", boxes_train.size());

        // === Positive ===
        // Create positive samples (ROI, resizing and reshaping)
        std::vector<svm_sample_type> samples_sgl;
        for (int i = 0; i < boxes_train.size(); i++) {
            for (int j = 0; j < boxes_train[i].size(); j++) {
                svm_sample_type sample;
                ConvertToSvmSample(imgs_train[i], boxes_train[i][j], sample,
                                   CHIP_SIZE);
                samples_sgl.push_back(sample);
            }
        }
        // Create corresponding labels
        std::vector<int> labels_sgl(samples_sgl.size(), i);  // fill label i
        // Append positive samples
        samples.insert(samples.end(), samples_sgl.begin(), samples_sgl.end());
        labels.insert(labels.end(), labels_sgl.begin(), labels_sgl.end());

        // === Negative ===
        // Negative boxes
        printf(" >> Create negative samples\n");
        std::vector<dlib::rectangle> boxes_train_neg;
        for (int s_idx = 0; s_idx < imgs_train.size(); s_idx++) {
            std::vector<dlib::rectangle> neg_boxes;
            CreateNegativeSamples(imgs_train[s_idx], boxes_train[s_idx],
                                  neg_boxes, neg_per_img);
            boxes_train_neg.insert(boxes_train_neg.end(), neg_boxes.begin(),
                                   neg_boxes.end());
        }
        printf("  > bboxes (neg) : %lu\n", boxes_train_neg.size());

        // Create negative samples (ROI, resizing and reshaping)
        std::vector<svm_sample_type> samples_sgl_neg;
        for (int i = 0; i < boxes_train_neg.size(); i++) {
            svm_sample_type sample;
            int img_idx = i / neg_per_img;
            ConvertToSvmSample(imgs_train[img_idx], boxes_train_neg[i], sample,
                               CHIP_SIZE);
            samples_sgl_neg.push_back(sample);
        }
        // Create corresponding labels
        std::vector<int> labels_sgl_neg(samples_sgl_neg.size(), -1);  // neg: -1
        // Append positive samples
        samples.insert(samples.end(), samples_sgl_neg.begin(),
                       samples_sgl_neg.end());
        labels.insert(labels.end(), labels_sgl_neg.begin(),
                      labels_sgl_neg.end());
    }

    // Normalize training data
    printf(" >> Normalize training data\n");
    dlib::vector_normalizer<svm_sample_type> normalizer;
    normalizer.train(samples);  // calculate mean and standard deviation
    for (int i = 0; i < samples.size(); i++) {
        samples[i] = normalizer(samples[i]);
    }

    // Change the order to use cross-validation
    dlib::randomize_samples(samples, labels);

    // Create multiclass SVM trainer
    dlib::svm_multiclass_linear_trainer<svm_kernel_type, int> trainer;
    trainer.set_c(svm_c);

    // Train
    printf(" >> Train\n");
    classifier.normalizer = normalizer;
    classifier.function = trainer.train(samples, labels);

    // Test (training data is already normalized)
    printf(" >> Test\n\n");
    dlib::matrix<double> test_ret = dlib::test_multiclass_decision_function(
        classifier.function, samples, labels);
    std::cout << test_ret << std::endl;

    empty = false;
}

// ===== exposed methods =====
SvmChipClassifier::SvmChipClassifier() { impl = new Impl; }
SvmChipClassifier::~SvmChipClassifier() { delete impl; }
void SvmChipClassifier::load(const std::string& filename) {
    impl->load(filename);
}
void SvmChipClassifier::save(const std::string& filename) {
    impl->save(filename);
}
int SvmChipClassifier::classify(const cv::Mat& src, const cv::Rect& rect) {
    return impl->classify(src, rect);
}
void SvmChipClassifier::train(const std::vector<std::string>& inxml_paths,
                              double svm_c, int neg_per_img) {
    impl->train(inxml_paths, svm_c, neg_per_img);
}

// ============================== ObjectDetector ===============================
void ObjectDetector::Impl::init() {
    printf("* Initialize Object Detector\n");

    // Initialize Multi-HOG
    for (int i = 0; i < N_OBJECT_DETECT; i++) {
        hog_detector.loadNext(OBJECT_DETECT_HOG_PATHS[i]);
    }
    // Initialize SVM
    classifier.load(OBJECT_CHIP_CLASSIFY_SVM_PATH);
    // Initialize Tracker
    prev_bboxes.resize(N_OBJECT_DETECT);
    tracking_states.resize(N_OBJECT_DETECT, false);
    trackers.resize(N_OBJECT_DETECT);
    // Initialize object 3D sizes
    objects_3d_points.resize(N_OBJECT_DETECT);
    for (int i = 0; i < N_OBJECT_DETECT; i++) {
        objects_3d_points[i].resize(4);
        float half_w = OBJECT_3D_SIZES[i][0] / 2.f;
        float half_h = OBJECT_3D_SIZES[i][1] / 2.f;
        objects_3d_points[i][0] = cv::Point3f(half_w, half_h, 0.f);
        objects_3d_points[i][1] = cv::Point3f(half_w, -half_h, 0.f);
        objects_3d_points[i][2] = cv::Point3f(-half_w, half_h, 0.f);
        objects_3d_points[i][3] = cv::Point3f(-half_w, -half_h, 0.f);
    }

    empty = false;
    interval_cnt = 0;
}

void ObjectDetector::Impl::detect(const cv::Mat& src,
                                  std::vector<std::vector<cv::Rect> >& bboxes) {
    bboxes.clear();
    bboxes.resize(N_OBJECT_DETECT);  // Create zero size vectors
    if (empty) {
        printf("* ObjectDetector is not initialized\n");
        return;
    }

    // === Detect in current frame ===
    // Convert OpenCV Mat to dlib
    dlib::array2d<dlib::bgr_pixel> dlib_img;
    ConvertMat(src, dlib_img);

    // Detect using HOG with some interval
    interval_cnt = (interval_cnt + 1) % HOG_INTERVAL;
    if (interval_cnt == 0) {
        std::vector<dlib::rect_detection> dets;
        hog_detector.impl->detect(dlib_img, dets);
        // For each detected rectangles
        int n_dets = dets.size();
        for (int i = 0; i < n_dets; i++) {
            dlib::rect_detection& det = dets[i];
            int label = det.weight_index;
            // Singulate labels
            if (DO_SINGULATE) {
                if (bboxes[label].size() > 0) continue;
            }
            // Classify chips
            int pred_label = classifier.impl->classify(dlib_img, det.rect);
            // Push to results
            if (label == pred_label) {
                bboxes[label].push_back(ConvertRectangle(det.rect));
            }
        }
    }

    // === Consider previous frame results ===
    for (int label = 0; label < N_OBJECT_DETECT; label++) {
        char& tracking_state = tracking_states[label];
        // Find objects which are not detected.
        if (prev_bboxes[label].size() > 0 && bboxes[label].size() == 0) {
            // Considering only highest probability object
            cv::Rect& prev_bbox = prev_bboxes[label][0];
            dlib::correlation_tracker& tracker = trackers[label];

            // == Tracking (translation only) ==
            // Note: I tried using OpenCV Tracking API but not good. (16/07/28)
            // Note: Dlib tracker is better due to scaling support. (16/08/02)

            // Initialize tracker
            if (!tracking_state) {
                dlib::array2d<dlib::bgr_pixel> dprev_frame;
                ConvertMat(prev_frame, dprev_frame);
                tracker.start_track(dprev_frame, ConvertRectangle(prev_bbox));
                tracking_state = true;
            }

            // Track
            dlib::array2d<dlib::bgr_pixel> dsrc;
            ConvertMat(src, dsrc);
            tracker.update(dsrc);
            dlib::drectangle drect = tracker.get_position();

            // Classify tracking results
            int pred_label = classifier.impl->classify(dsrc, drect);
            if (label == pred_label) {
                // Append
                bboxes[label].push_back(ConvertRectangle(drect));
            }
        } else {
            tracking_state = false;
        }
    }

    // === Update temporal data ===
    prev_frame = src.clone();
    prev_bboxes = bboxes;
}

void ObjectDetector::Impl::detect(
    const cv::Mat& src, std::vector<std::vector<cv::Rect> >& bboxes,
    std::vector<std::vector<cv::Point3f> >& positions) {
    // Detect in 2D
    detect(src, bboxes);

    positions.resize(N_OBJECT_DETECT);
    for (int label = 0; label < N_OBJECT_DETECT; label++) {
        // Back-project to 3D
        int n = bboxes[label].size();
        positions[label].resize(n);
        for (int i = 0; i < n; i++) {
            // Rect -> vector<Point2f>
            std::vector<cv::Point2f> object_2d_points(4);
            cv::Rect& bbox = bboxes[label][i];
            float half_w = bbox.width / 2.f;
            float half_h = bbox.height / 2.f;
            float x = float(bbox.x) + half_w;
            float y = float(bbox.y) + half_h;
            object_2d_points[0] = cv::Point2f(x + half_w, y + half_h);
            object_2d_points[1] = cv::Point2f(x + half_w, y - half_h);
            object_2d_points[2] = cv::Point2f(x - half_w, y + half_h);
            object_2d_points[3] = cv::Point2f(x - half_w, y - half_h);

            // Back-project (CV_P3P mode uses 4 points)
            cv::Mat rvec, tvec;
            bool ret = cv::solvePnP(objects_3d_points[i], object_2d_points,
                                    getWebCameraIntrinsicMatrix(), cv::Mat(),
                                    rvec, tvec, false, CV_P3P);
            if (!ret) {
                bboxes[label].erase(bboxes[label].begin() + i);
                positions[label].erase(positions[label].begin() + i);
                continue;
            }

            // Register (fix coordinates)
            positions[label][i].x = float(tvec.at<double>(0)) * -1.f;
            positions[label][i].y = float(tvec.at<double>(1)) * -1.f;
            positions[label][i].z = float(tvec.at<double>(2));

            // Convert coordinates from camera to world
            positions[label][i] = cvtWebCameraCoord2World(positions[label][i]);
        }
    }
}

void ObjectDetector::Impl::drawDebug(
    cv::Mat& img, const std::vector<std::vector<cv::Rect> >& bboxes) {
    for (int label = 0; label < N_OBJECT_DETECT; label++) {
        for (int i = 0; i < bboxes[label].size(); i++) {
            cv::rectangle(img, bboxes[label][i], OBJECT_VIEW_COLORS[label], 2);
            // TODO Add label drawing
        }
    }
}

// ===== exposed methods =====
ObjectDetector::ObjectDetector() { impl = new Impl; }
ObjectDetector::~ObjectDetector() { delete impl; }
void ObjectDetector::init() { impl->init(); }
void ObjectDetector::detect(const cv::Mat& src,
                            std::vector<std::vector<cv::Rect> >& bboxes) {
    impl->detect(src, bboxes);
}
void ObjectDetector::detect(const cv::Mat& src,
                            std::vector<std::vector<cv::Rect> >& bboxes,
                            std::vector<std::vector<cv::Point3f> >& positions) {
    impl->detect(src, bboxes, positions);
}
void ObjectDetector::drawDebug(
    cv::Mat& img, const std::vector<std::vector<cv::Rect> >& bboxes) {
    impl->drawDebug(img, bboxes);
}
