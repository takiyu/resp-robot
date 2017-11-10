#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <string>

#include "../modules/robovie/robovie_serial.h"
#include "../viewer/render/gl_imgui.h"
#include "../viewer/render/gl_window.h"

int main(int argc, char *argv[]) {
    // Arguments
    bool safty_clamp = true;
    bool degree_mode = true;
    bool debug_log = false;
    for (int i = 1; i < argc; i++) {
        if (std::string(argv[i]) == "--clamp") {
            safty_clamp ^= 1;
        } else if (std::string(argv[i]) == "--normalize") {
            degree_mode ^= 1;
        } else if (std::string(argv[i]) == "--log") {
            debug_log ^= 1;
        } else if (std::string(argv[i]) == "-h") {
            printf("--clamp\n");
            printf("--normalize\n");
            printf("--log\n");
            return 0;
        } else {
            printf("Invalid parameter: %s\n", argv[i]);
            return 1;
        }
    }
    printf("safty clamp: %s\n", safty_clamp ? "true" : "false");
    printf("degree_mode: %s\n", degree_mode ? "true" : "false");
    printf("debug_log: %s\n", debug_log ? "true" : "false");

    // Window
    GLWindow window(1024, 512);
    bool gl_ret = window.init("title", true, 1);
    if (!gl_ret) return false;
    InitImGui(window);

    // Robovie
    RobovieSerial robovie_serial(degree_mode, safty_clamp, debug_log);
    robovie_serial.open();

    printf("* Start main loop\n");
    while (!window.shouldClose()) {
        window.activate();
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        NewImGuiFrame();

        robovie_serial.drawGlMotorUi();

        RenderImGuiFrame();
        UpdateImGuiInput(window);
        window.update();
    }

    ExitImGui();
}
