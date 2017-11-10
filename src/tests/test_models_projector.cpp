#include <stdio.h>

#include <boost/thread.hpp>

#include "../fps.h"
#include "../models/projector.h"
#include "../modules/modules.h"
#include "../viewer/render/gl_imgui.h"
#include "../viewer/shapes_viewer.h"

int main(int argc, char const* argv[]) {
    // Shapes viewer
    ShapesViewer shapes_viewer(1024, 512);
    shapes_viewer.init("viewer", 1);
    InitImGui(shapes_viewer);

    // Modules and Projector
    Modules modules;
    RobovieProjector projector(&modules);

    modules.init();
    modules.initGlUi();

    // Behavior container
    ThreadCopiablePtr<MovementCtr> behavior_ctr;  // src

    // Behavior thread
    boost::thread* update_thread = new boost::thread([&]() {
        printf("* Start a updating thread\n");
        while (true) {
            auto boost_sleep_ms = boost::posix_time::milliseconds(2000);
            boost::this_thread::sleep(boost_sleep_ms);
            // Set behavior values
            behavior_ctr.reset();  // reset
            behavior_ctr->voice.set("あいうえお", 1.f);
        }
    });

    // Fps
    FpsCounter fps;

    printf("* Start main loop\n");
    while (!shapes_viewer.shouldClose()) {
        // Setup window
        shapes_viewer.activate();
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        NewImGuiFrame();

        modules.update();

        // Copy to result container
        behavior_ctr.lock();
        MovementCtr result_ctr = behavior_ctr->use();
        behavior_ctr.unlock();

        // Project
        projector.project(result_ctr);

        // Draw UIs
        modules.drawGlUi();

        // Update window
        RenderImGuiFrame();
        UpdateImGuiInput(shapes_viewer);
        shapes_viewer.update();

        // Show fps
        std::cout << fps.update() << std::endl;
    }
    ExitImGui();

    printf("* Exit updating thread\n");
    update_thread->interrupt();
    update_thread->join();
    delete update_thread;

    modules.exit();

    return 0;
}
