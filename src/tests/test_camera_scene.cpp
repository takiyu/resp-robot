#include <stdio.h>

#include "../fps.h"
#include "../modules/camera/camera_scene.h"
#include "../viewer/render/gl_imgui.h"
#include "../viewer/shapes_viewer.h"

int main(int argc, char const* argv[]) {
    // Shapes viewer
    ShapesViewer shapes_viewer(1024, 512);
    shapes_viewer.init("viewer", 1);
    InitImGui(shapes_viewer);

    // Camera scene
    CameraScene camera_scene;
    camera_scene.init();
    camera_scene.initGlUi();

    // Fps
    FpsCounter fps;

    printf("* Start main loop\n");
    while (!shapes_viewer.shouldClose()) {
        // Setup window
        shapes_viewer.activate();
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        NewImGuiFrame();

        // Draw UI
        camera_scene.drawGlUi();

        // Update window
        RenderImGuiFrame();
        UpdateImGuiInput(shapes_viewer);
        shapes_viewer.update();

        // Show fps
        std::cout << fps.update() << std::endl;

        // OpenCV frame
        // camera_scene.showDebugFrame();
        // char k = cv::waitKey(1);
        // if (k == 'q') {
        //     break;
        // }
    }

    camera_scene.exit();
    ExitImGui();

    return 0;
}
