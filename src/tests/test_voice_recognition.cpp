#include <stdio.h>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>

#include "../modules/audio/audio.h"
#include "../modules/audio/voice_recog.h"  // must be the after of including glm
#include "../viewer/render/gl_imgui.h"
#include "../viewer/render/gl_window.h"

int main(int argc, char const* argv[]) {
    InitializePortAudio();

    // Voice Recognition
    VoiceRecognition voice_recog;
    voice_recog.init();

    // Window
    GLWindow window(1024, 512);
    bool gl_ret = window.init("title", true, 1);
    if (!gl_ret) return false;
    InitImGui(window);

    // Main loop
    printf("* Start main loop\n");
    bool exit_flag = false;
    bool prev_recog_active = false;
    while (!window.shouldClose()) {
        window.activate();
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        NewImGuiFrame();

        // Output recognition state
        bool recog_active = voice_recog.isProcessing();
        if (!prev_recog_active && recog_active) printf("Recog Voice start\n");
        if (prev_recog_active && !recog_active) printf("Recog Voice end\n");
        prev_recog_active = recog_active;

        // Output recognition results
        while (voice_recog.hasResult()) {
            std::string res = voice_recog.popResult();
            printf("result: %s\n", res.c_str());
            if (res.find("終") != std::string::npos) exit_flag = true;
        }
        if (exit_flag) break;

        // Draw GL Ui
        voice_recog.drawGlUi();

        RenderImGuiFrame();
        UpdateImGuiInput(window);
        window.update();
    }
    printf("exit\n");
    ExitImGui();

    voice_recog.exit();
    TerminatePortAudio();
    return 0;
}
