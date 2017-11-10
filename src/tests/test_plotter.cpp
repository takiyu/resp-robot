#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>

#include <stdio.h>
#include <sstream>
#include <string>
#include <vector>

#include "../fps.h"
#include "../viewer/plotter.h"
#include "../viewer/render/gl_imgui.h"
#include "../viewer/shapes_viewer.h"

#include "imgui/imgui.h"

int main(int argc, char const* argv[]) {
    // Shapes viewer
    ShapesViewer shapes_viewer(1024, 512);
    shapes_viewer.init("viewer", 1);
    InitImGui(shapes_viewer);

    // Single plotter
    Plotter plotter_one;
    plotter_one.initGlUi();

    // Time-series plotter
    TimeSeriesPlotter plotter_series;
    plotter_series.initGlUi();

    // Time-series plotters
    std::vector<TimeSeriesPlotter> plotters(7);
    for (int i = 0; i < plotters.size(); i++) {
        plotters[i].initGlUi();
        plotters[i].setSize(300, 300);
        std::stringstream ss;
        ss << "title " << i;
        plotters[i].setTitle(ss.str());
        plotters[i].setHistorySec(3.f);
    }

    // Create static plotting data
    std::vector<float> xs, ys1, ys2;
    for (int i = 0; i < 30; i++) {
        xs.push_back(static_cast<float>(i));
        ys1.push_back(static_cast<float>(i * i));
        ys2.push_back(static_cast<float>(i * i * i));
    }
    plotter_one.setData("A", xs, ys1);
    plotter_one.setData("B", xs, ys2);

    // Fps
    FpsCounter fps;

    clock_t first_clock = clock();
    unsigned int cnt = 0;
    printf("* Rendering Loop\n");
    while (!shapes_viewer.shouldClose()) {
        // Setup window
        shapes_viewer.activate();
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        NewImGuiFrame();

        // Set plotting data (plotter_series)
        float sec = static_cast<float>(clock() - first_clock) / CLOCKS_PER_SEC;
        plotter_series.appendData("C", cnt);
        plotter_series.setXRange(-sec, 0.f);

        // Set plotting data (plotters)
        for (int i = 0; i < plotters.size(); i++) {
            plotters[i].appendData("C", cnt);
        }

        // Draw plotter
        ImGui::Begin("Plotter 1", NULL, ImGuiWindowFlags_AlwaysAutoResize);
        plotter_one.drawGlUi();
        ImGui::End();

        // Draw time-series plotter
        ImGui::Begin("Plotter 2", NULL, ImGuiWindowFlags_AlwaysAutoResize);
        plotter_series.drawGlUi();
        ImGui::End();

        // Draw time-series plotters
        ImGui::Begin("Plotter 3", NULL, ImGuiWindowFlags_AlwaysAutoResize);
        ImGui::Columns(2);
        for (int i = 0; i < plotters.size(); i++) {
            plotters[i].drawGlUi();
            if (i == plotters.size() / 2) ImGui::NextColumn();
        }
        ImGui::End();

        // Update window
        RenderImGuiFrame();
        UpdateImGuiInput(shapes_viewer);
        shapes_viewer.update();

        std::cout << fps.update() << std::endl;
        cnt++;
    }

    ExitImGui();
    printf("* Exit\n");

    return 0;
}
