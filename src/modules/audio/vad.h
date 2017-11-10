#ifndef VAD_160624
#define VAD_160624

#include <portaudio.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <chrono>

#include "webrtc_vad.h"

#include "audio.h"

#include "../../config.h"

class VAD {
public:
    VAD(int cache_size = 50) : stream(NULL), ACT_CACHE_SIZE(cache_size) {
        data.vad = NULL;
    }
    ~VAD() { exit(); }

    //// Initialization
    // rate: 8000, 16000 or 32000 Hz
    // frame_ms: 10, 20 or 30 ms
    // mode: 0, 1, 2, or 3
    bool init(int rate = 8000, int frame_ms = 30, int mode = 3);
    void exit();

    void enable() { data.enabled = true; }
    void disable() { data.enabled = false; }

    // Mic input scale
    void setMicScale(float scale) { data.mic_scale = scale; }
    // Min non-active milliseconds
    void setMinNonactiveMs(float ms) { min_nonactive_ms = ms; }

    // change of status
    void updateStatus();
    bool isVoiceActive() { return voice_active; }
    bool isVoiceStarted() { return !prev_voice_active && voice_active; }
    bool isVoiceFinished() { return prev_voice_active && !voice_active; }
    float getVoiceLengthMs();

    // GL UI
    void drawGlUi();

private:
    struct PaVadData {
        bool enabled;
        int activity;  // 1:Active, 0:Non-active, -1:Error
        VadInst *vad;
        int sampling_rate;
        int frame_length;
        float mic_scale;
    };

    struct PaVadData data;
    PaStream *stream;

    void startStream();
    void stopStream();

    static int paCallback(const void *input_buffer, void *output_buffer,
                          unsigned long frames_per_buffer,
                          const PaStreamCallbackTimeInfo *time_info,
                          PaStreamCallbackFlags status_flags, void *user_data);

    // status
    bool voice_active, prev_voice_active;
    std::chrono::system_clock::time_point last_active_clock;
    float min_nonactive_ms;
    // voice length
    std::chrono::system_clock::time_point voice_start_clock;

    // Cache for drawing
    const int ACT_CACHE_SIZE;
    int activity_cache_idx;
    std::vector<float> activity_cache, raw_activity_cache;
};
#endif
