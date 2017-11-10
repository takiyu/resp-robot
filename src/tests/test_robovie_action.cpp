#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <string>

#include "../modules/robovie/robovie_action.h"
#include "../viewer/render/gl_imgui.h"
#include "../viewer/render/gl_window.h"

int main(int argc, char *argv[]) {
    // Window
    GLWindow window(1024, 512);
    bool gl_ret = window.init("title", true, 1);
    if (!gl_ret) return false;
    InitImGui(window);

    // Robovie
    RobovieAction robovie_action;
    robovie_action.open();

    printf("* Start main loop\n");
    while (!window.shouldClose()) {
        // Setup window
        window.activate();
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        NewImGuiFrame();

        // Draw UIs
        robovie_action.drawGlMotorUi();
        robovie_action.drawGlActionUi();

        // Update window
        RenderImGuiFrame();
        UpdateImGuiInput(window);
        window.update();
    }

    ExitImGui();
}
