#include <stdio.h>

#include "../fps.h"
#include "../models/weights_optimizer.h"
#include "../viewer/render/gl_imgui.h"
#include "../viewer/shapes_viewer.cpp"

int main(int argc, char const* argv[]) {
    printf("** Behavior weights optimizing tool\n");

    // Shapes viewer
    ShapesViewer shapes_viewer(1024, 512);
    shapes_viewer.init("viewer", 1);
    InitImGui(shapes_viewer);

    // Optimizer
    BehaviorWeightsOptimizer optimizer;
    optimizer.init();
    optimizer.initGlUi();

    // Fps
    FpsCounter fps;

    printf("* Start main loop\n");
    while (!shapes_viewer.shouldClose()) {
        // Setup window
        shapes_viewer.activate();
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        NewImGuiFrame();

        // Draw UIs
        optimizer.drawGlUi();

        // Update window
        RenderImGuiFrame();
        UpdateImGuiInput(shapes_viewer);
        shapes_viewer.update();

        // Show fps
        // std::cout << fps.update() << std::endl;
        fps.update();
    }

    optimizer.exit();

    ExitImGui();

    return 0;
}
