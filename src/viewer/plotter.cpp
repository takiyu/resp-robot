#include "plotter.h"

#include <limits>
#include <utility>

#include <GL/glew.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/filesystem.hpp>
#include <boost/thread.hpp>

// Impl pattern is needed for once including of gnuplot-iostream
#define GNUPLOT_DEPRECATE_WARN
#include "gnuplot-iostream/gnuplot-iostream.h"

#include "imgui/imgui.h"
#include "render/gl_utils.h"

#include "../config.h"

namespace {
std::string GenTempFilename(
    const std::string& exp = "tmp-plotter-%%%%-%%%%-%%%%-%%%%.png") {
    return boost::filesystem::unique_path(
               boost::filesystem::temp_directory_path() / exp)
        .c_str();
}

void FindYRange(const std::map<std::string, PlotData>& plot_data_map,
                float& min_v, float& max_v) {
    min_v = std::numeric_limits<float>::max();
    max_v = std::numeric_limits<float>::min();

    // Search
    std::map<std::string, PlotData>::const_iterator itr = plot_data_map.begin();
    for (; itr != plot_data_map.end(); itr++) {
        const std::vector<float>& ys = itr->second.second;
        for (int i = 0; i < ys.size(); i++) {
            if (ys[i] < min_v) min_v = ys[i];
            if (max_v < ys[i]) max_v = ys[i];
        }
    }
}
}

// === Impl Class ===
class Plotter::Impl {
public:
    Impl()
        : TMP_FILENAME(GenTempFilename()),
          gp_width(-1.f),
          gp_height(-1.f),
          gp_xrange(false),
          gp_yrange(false),
          gp_linewidth(-1.f),
          plot_frame_width(100),  // dummy initial size > 0
          plot_frame_height(100) {}
    ~Impl() {
        if (boost::filesystem::exists(TMP_FILENAME)) {
            boost::filesystem::remove(TMP_FILENAME);  // remove tmpfile
        }
    }

    void setTitle(const std::string& title);
    void setSize(float width, float height);
    void setXRange(float x_min, float x_max);
    void setYRange(float y_min, float y_max);
    void setLineWidth(float width);
    void setData(const std::string label, const std::vector<float>& xs,
                 const std::vector<float>& ys);

    // GL UIs
    void initGlUi();
    void drawGlUi();

private:
    const std::string TMP_FILENAME;

    Gnuplot gp;
    std::string gp_title;
    float gp_width, gp_height;
    bool gp_xrange, gp_yrange;
    float gp_xrange_min, gp_xrange_max, gp_yrange_min, gp_yrange_max;
    float gp_linewidth;
    std::map<std::string, PlotData> plot_data_map;

    clock_t prev_clock;

    cv::Mat plot_frame;
    GLuint plot_frame_tex_id;
    GLsizei plot_frame_width, plot_frame_height;
};

// === Impl Methods ===
void Plotter::Impl::setTitle(const std::string& title) { gp_title = title; }

void Plotter::Impl::setSize(float width, float height) {
    gp_width = width;
    gp_height = height;
}

void Plotter::Impl::setXRange(float x_min, float x_max) {
    gp_xrange = true;
    gp_xrange_min = x_min;
    gp_xrange_max = x_max;
}

void Plotter::Impl::setYRange(float y_min, float y_max) {
    gp_yrange = true;
    gp_yrange_min = y_min;
    gp_yrange_max = y_max;
}

void Plotter::Impl::setLineWidth(float width) { gp_linewidth = width; }

void Plotter::Impl::setData(const std::string label,
                            const std::vector<float>& xs,
                            const std::vector<float>& ys) {
    plot_data_map[label] = PlotData(xs, ys);
}

void Plotter::Impl::initGlUi() {
    // Create dummy image
    plot_frame = cv::Mat::zeros(plot_frame_height, plot_frame_width, CV_8UC3) +
                 cv::Scalar(0, 255, 0);  // green
    // Create texture
    glGenTextures(1, &plot_frame_tex_id);
    glBindTexture(GL_TEXTURE_2D, plot_frame_tex_id);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    // internal_format:RGB, format:BGR
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, plot_frame_width, plot_frame_height,
                 0, GL_BGR, GL_UNSIGNED_BYTE, plot_frame.data);

    // set first clock
    prev_clock = clock();
}

void Plotter::Impl::drawGlUi() {
    // === GnuPlot ===
    gp.clearTmpfiles();  // clear previous temp file

    if (!plot_data_map.empty()) {
        // Wait for drawing
        clock_t curr_clock = clock();
        double elapse = (double)(curr_clock - prev_clock) / CLOCKS_PER_SEC;
        double sleep_ms = PLOTTER_PNG_WAIT_MS - elapse * 1000.0;
        if (sleep_ms > 0) {
            auto boost_sleep_ms = boost::posix_time::milliseconds(sleep_ms);
            boost::this_thread::sleep(boost_sleep_ms);
        }

        // Load previous graph from temporary file
        cv::Mat plot_frame_tmp = cv::imread(TMP_FILENAME);
        if (!plot_frame_tmp.empty()) {
            plot_frame = plot_frame_tmp;
        }

        // Set output png and size
        gp << "set terminal png";
        if (gp_width > 0.f && gp_height > 0.f) {
            gp << " size " << gp_width << "," << gp_height;
        }
        gp << std::endl;
        // Set title
        if (!gp_title.empty()) {
            gp << "set title '" << gp_title << "'" << std::endl;
        }
        // Set output filename
        gp << "set output '" << TMP_FILENAME << "'" << std::endl;
        // Set x range
        if (gp_xrange) {
            gp << "set xrange [" << gp_xrange_min << ":" << gp_xrange_max << "]"
               << std::endl;
        }
        // Set y range
        if (gp_yrange) {
            gp << "set yrange [" << gp_yrange_min << ":" << gp_yrange_max << "]"
               << std::endl;
        } else {
            // Escape `Warning: empty y range [0:0], adjusting to [-1:1]`
            float min_v, max_v;
            FindYRange(plot_data_map, min_v, max_v);
            if (max_v - min_v > 1e-6) {
                gp << "set yrange [" << min_v << ":" << max_v << "]"
                   << std::endl;
            } else {
                gp << "set yrange [-1: 1]" << std::endl;
            }
        }
        // Plot each data
        std::map<std::string, PlotData>::iterator itr = plot_data_map.begin();
        gp << "plot";
        for (; itr != plot_data_map.end(); itr++) {
            gp << gp.file1d(itr->second) << "with lines";
            if (gp_linewidth > 0.f) {
                gp << " linewidth " << gp_linewidth;
            }
            gp << " title '" << itr->first << "'";
            gp << ",";
        }
        gp << std::endl;

        // Update previous clock
        prev_clock = curr_clock;
    }

    // === plotter frame texture (ImGui) ===
    if (!plot_frame.empty()) {
        // Update texture
        glBindTexture(GL_TEXTURE_2D, plot_frame_tex_id);
        if (plot_frame.rows == plot_frame_height &&
            plot_frame.cols == plot_frame_width) {
            // update data only
            // format:BGR
            glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, plot_frame_width,
                            plot_frame_height, GL_BGR, GL_UNSIGNED_BYTE,
                            plot_frame.data);
        } else {
            // update texture size and data
            plot_frame_height = plot_frame.rows;
            plot_frame_width = plot_frame.cols;
            // internal_format:RGB, format:BGR
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, plot_frame_width,
                         plot_frame_height, 0, GL_BGR, GL_UNSIGNED_BYTE,
                         plot_frame.data);
        }
        checkGlError();

        // image
        ImTextureID imgui_tex_id = (void*)(intptr_t)plot_frame_tex_id;
        ImGui::Image(imgui_tex_id, ImVec2(plot_frame_width, plot_frame_height),
                     ImVec2(0, 0), ImVec2(1, 1), ImColor(255, 255, 255, 255),
                     ImColor(0, 0, 0, 0));
    }
}

// === Exposed Methods ===
Plotter::Plotter() { impl = new Impl; }
Plotter::~Plotter() { delete impl; }
void Plotter::setTitle(const std::string& title) { impl->setTitle(title); }
void Plotter::setSize(float width, float height) {
    impl->setSize(width, height);
}
void Plotter::setXRange(float x_min, float x_max) {
    impl->setXRange(x_min, x_max);
}
void Plotter::setYRange(float y_min, float y_max) {
    impl->setYRange(y_min, y_max);
}
void Plotter::setLineWidth(float width) { impl->setLineWidth(width); }
void Plotter::setData(const std::string label, const std::vector<float>& xs,
                      const std::vector<float>& ys) {
    impl->setData(label, xs, ys);
}
void Plotter::initGlUi() { impl->initGlUi(); }
void Plotter::drawGlUi() { impl->drawGlUi(); }

// ============================ Time-series Ploter =============================
void TimeSeriesPlotter::setHistorySec(float sec) {
    history_sec = sec;
    setXRange(-sec, 0.f);
}

void TimeSeriesPlotter::appendData(const std::string& label, float value) {
    std::vector<float>& xs = series_data[label].first;
    std::vector<float>& ys = series_data[label].second;

    // Append value
    clock_t curr_clock = clock();
    double curr_sec = static_cast<double>(curr_clock) / CLOCKS_PER_SEC;
    xs.push_back(static_cast<float>(curr_sec));
    ys.push_back(value);

    // Remove old entries
    if (history_sec > 0.f) {
        int valid_idx = 0;
        for (; valid_idx < xs.size(); valid_idx++) {
            if (curr_sec - xs[valid_idx] < history_sec) break;
        }
        xs.erase(xs.begin(), xs.begin() + valid_idx);
        ys.erase(ys.begin(), ys.begin() + valid_idx);
    }

    // Convert relative time stamps
    std::vector<float> xs_rel(xs.size());
    for (int i = 0; i < xs.size(); i++) {
        xs_rel[i] = xs[i] - curr_sec;
    }

    // Set to plotter
    setData(label, xs_rel, ys);
}
