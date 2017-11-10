#include <stdio.h>

#include "../modules/audio/audio.h"
#include "../modules/audio/vad.h"
#include "../viewer/render/gl_imgui.h"
#include "../viewer/render/gl_window.h"

int main(int argc, char const *argv[]) {
    InitializePortAudio();

    // VAD
    VAD vad;
    vad.init();

    // Window
    GLWindow window(1024, 512);
    bool gl_ret = window.init("title", true, 1);
    if (!gl_ret) return false;
    InitImGui(window);

    // Main loop
    printf("* Start main loop\n");
    while (!window.shouldClose()) {
        window.activate();
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        NewImGuiFrame();

        // Update VAD status
        vad.updateStatus();

        // Print voice status
        if (vad.isVoiceStarted()) {
            printf("Voice start\n");
        } else if (vad.isVoiceFinished()) {
            printf("Voice finish\n");
            printf(" >> length: %f ms\n", vad.getVoiceLengthMs());
        }

        // Draw GL Ui
        vad.drawGlUi();

        RenderImGuiFrame();
        UpdateImGuiInput(window);
        window.update();
    }

    ExitImGui();

    vad.exit();
    TerminatePortAudio();
    return 0;
}
