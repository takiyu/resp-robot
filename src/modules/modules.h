#ifndef MODULES_H_160913
#define MODULES_H_160913

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>

#include "audio/audio.h"
#include "audio/vad.h"
#include "audio/voice_recog.h"  // must be the after of including glm
#include "audio/voice_synthesis.h"
#include "camera/camera_scene.h"
#include "robovie/robovie_action.h"

class Modules {
public:
    Modules() : inited(false) {}
    ~Modules() {
        if (inited) exit();
    }

    void init();
    void exit();
    void update();

    // GL UIs
    void initGlUi();
    void drawGlUi();

    // Modules
    CameraScene camera_scene;
    RobovieAction robovie_action;
//     VAD vad;
//     VoiceRecognition voice_recog;
//     VoiceSynthesis voice_synthesis;
//     WavPlayer wav_player;

private:
    bool inited;
};

#endif
