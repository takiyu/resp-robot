#include <stdio.h>
#include <unistd.h>

#include "../modules/audio/audio.h"
#include "../modules/audio/voice_recog.h"
#include "../modules/audio/voice_synthesis.h"

int main(int argc, char const* argv[]) {
    InitializePortAudio();

    VoiceRecognition voice_recog;
    voice_recog.init();

    VoiceSynthesis voice_synthesis;
    voice_synthesis.init();

    WavPlayer player;

    // Main loop
    printf("* Start main loop\n");
    bool exit_flag = false;
    bool prev_recog_active = false;
    while (true) {
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
            // echo
            if (res.size() > 0) voice_synthesis.synthesis(res, player, false);
        }
        if (exit_flag) break;
        usleep(10000);
    }

    printf("exit\n");
    voice_recog.exit();
    voice_synthesis.exit();

    TerminatePortAudio();
    return 0;
}
