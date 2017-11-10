#include <stdio.h>

#include "fps.h"
#include "models/resp_model.h"
#include "viewer/render/gl_imgui.h"
#include "viewer/shapes_viewer.cpp"

int main(int argc, char const* argv[]) {
    // Shapes viewer
    ShapesViewer shapes_viewer(1024, 512);
    shapes_viewer.init("viewer", 1);
    InitImGui(shapes_viewer);

    // Model
    ResponsiveModel resp_model;
    resp_model.init();
    resp_model.initGlUi();

    // Fps
    FpsCounter fps;

    printf("* Start main loop\n");
    while (!shapes_viewer.shouldClose()) {
        // Setup window
        shapes_viewer.activate();
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        NewImGuiFrame();

        // Update responsive model
        resp_model.update();

        // Draw UIs
        resp_model.drawGlUi();

        // Update window
        RenderImGuiFrame();
        UpdateImGuiInput(shapes_viewer);
        shapes_viewer.update();

        // Show fps
        // std::cout << fps.update() << std::endl;
        fps.update();
    }
    resp_model.exit();

    ExitImGui();

    return 0;
}
