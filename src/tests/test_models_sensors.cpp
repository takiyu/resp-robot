#include <stdio.h>

#include <boost/thread.hpp>

#include "../fps.h"
#include "../models/sensors.h"
#include "../modules/modules.h"
#include "../viewer/render/gl_imgui.h"
#include "../viewer/shapes_viewer.h"

int main(int argc, char const* argv[]) {
    // Shapes viewer
    ShapesViewer shapes_viewer(1024, 512);
    shapes_viewer.init("viewer", 1);
    InitImGui(shapes_viewer);

    // Modules and Sensors
    Modules modules;
    Sensors sensors(&modules);

    modules.init();
    modules.initGlUi();

    // Result container
    ThreadCopiablePtr<SensorCtr> result_ctr;

    // Update thread
    boost::thread* update_thread = new boost::thread([&]() {
        printf("* Start a updating thread\n");
        while (true) {
            auto boost_sleep_ms = boost::posix_time::milliseconds(100);
            boost::this_thread::sleep(boost_sleep_ms);
            // Get sensor values
            modules.update();
            ThreadCopiablePtr<SensorCtr> sensor_ctr_local;
            sensors.get(*sensor_ctr_local);
            // Copy to result container
            result_ctr = sensor_ctr_local;
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

        printf(" [face_valids]: ");
        for (int i = 0; i < result_ctr->face_valids.size(); i++) {
            printf(" %s ", result_ctr->face_valids[i] ? "o" : "x");
        }
        printf(", [object_valids]: ");
        for (int i = 0; i < result_ctr->object_valids.size(); i++) {
            printf(" %s ", result_ctr->object_valids[i] ? "o" : "x");
        }
        printf("\n");

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
