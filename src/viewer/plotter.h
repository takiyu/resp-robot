#ifndef PLOTTER_161122
#define PLOTTER_161122

#include <map>
#include <string>
#include <vector>

typedef std::pair<std::vector<float>, std::vector<float> > PlotData;

class Plotter {
public:
    Plotter();
    ~Plotter();

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
    class Impl;
    Impl* impl;
};

class TimeSeriesPlotter : public Plotter {
public:
    TimeSeriesPlotter() : history_sec(-1.f) {}
    ~TimeSeriesPlotter() {}

    void setHistorySec(float sec);
    void appendData(const std::string& label, float value);

private:
    float history_sec;
    std::map<std::string, PlotData> series_data;  // [sec, value]
};

#endif
