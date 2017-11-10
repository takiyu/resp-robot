#include "vad.h"

#include "imgui/imgui.h"

bool VAD::init(int rate, int frame_ms, int mode) {
    printf("* Initialize VAD\n");
    printf(" >> WebRtc VAD\n");

    // Create instance
    if (data.vad == NULL && (WebRtcVad_Create(&(data.vad)) < 0)) {
        printf("* Failed to create VAD instance.\n");
        return false;
    }

    // Set values
    data.enabled = true;
    data.activity = 0;
    if (WebRtcVad_Init(data.vad) < 0) {
        printf(" >> Failed to initialize VAD instance\n");
        return false;
    }
    data.sampling_rate = rate;
    data.frame_length = rate * frame_ms / 1000;
    data.mic_scale = VAD_MIC_SCALE_DEFALUT;
    min_nonactive_ms = VAD_MIN_NONACTIVE_MS;

    // Check frame length
    if (WebRtcVad_ValidRateAndFrameLength(data.sampling_rate,
                                          data.frame_length) < 0) {
        printf(" >> Invalid sampling rate or frame_length\n");
        return false;
    }

    // Set mode
    if (0 <= mode and mode <= 3) {
        WebRtcVad_set_mode(data.vad, mode);
        printf(" >> Set mode (%d)\n", mode);
    } else {
        printf(" >> Invalid mode (%d)\n", mode);
    }

    startStream();

    // Cache array
    activity_cache.resize(ACT_CACHE_SIZE, 0);
    raw_activity_cache.resize(ACT_CACHE_SIZE, 0);
    activity_cache_idx = 0;

    return true;
}

void VAD::exit() {
    stopStream();
    if (data.vad != NULL) {
        printf("* Release WebRtc VAD instance\n");
        WebRtcVad_Free(data.vad);
        data.vad = NULL;
    }
}

void VAD::updateStatus() {
    if (data.activity == -1) {
        printf("* VAD Error\n");
        return;
    }
    // now clock
    std::chrono::system_clock::time_point now_clock =
        std::chrono::system_clock::now();

    // === Fill nonactive hole ===
    if (data.activity) {
        last_active_clock = std::chrono::system_clock::now();
    }
    float elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                           now_clock - last_active_clock)
                           .count();

    // Set previous activity
    prev_voice_active = voice_active;
    // Set current activity
    if (data.activity || elapsed_ms < min_nonactive_ms) {
        voice_active = true;
    } else {
        voice_active = false;
    }

    // === Voice length ===
    if (isVoiceStarted()) {
        voice_start_clock = std::chrono::system_clock::now();
    }
}

float VAD::getVoiceLengthMs() {
    // clock
    float length_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                          last_active_clock - voice_start_clock)
                          .count();
    return length_ms;
}

void VAD::drawGlUi() {
    // Update cache
    activity_cache[activity_cache_idx] = static_cast<float>(voice_active);
    raw_activity_cache[activity_cache_idx] = static_cast<float>(data.activity);
    activity_cache_idx = (activity_cache_idx + 1) % ACT_CACHE_SIZE;

    // Common Window
    ImGui::Begin("Modules");
    // Header
    if (ImGui::CollapsingHeader("VAD", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("Status: " + voice_active ? "Active" : "Nonactive");
        ImGui::DragFloat("mic scale##vad", &(data.mic_scale), 0.1f);
        ImGui::DragFloat("min nonactive ms##vad", &(min_nonactive_ms));
        ImGui::PlotLines("Activity", &activity_cache[0], ACT_CACHE_SIZE,
                         activity_cache_idx, NULL, 0.0f, 1.0f, ImVec2(0, 40));
        ImGui::PlotLines("Activity (Raw)", &raw_activity_cache[0],
                         ACT_CACHE_SIZE, activity_cache_idx, NULL, 0.0f, 1.0f,
                         ImVec2(0, 40));
    }
    ImGui::End();
}

void VAD::startStream() {
    printf("* Open VAD I/O stream\n");
    if (stream != NULL) {
        printf(" >> already opened\n");
        return;
    }

    // Get device parameters
    PaStreamParameters input_params;
    GetPortAudioStreamParams(input_params, 1, paInt16,
                             (double)data.sampling_rate, true);

    // Open
    PaError err =
        Pa_OpenStream(&stream, &input_params, NULL, (double)data.sampling_rate,
                      data.frame_length, paNoFlag, VAD::paCallback, &data);
    CheckPaError(err);
    // Start
    err = Pa_StartStream(stream);
    CheckPaError(err);

    // Status
    voice_active = false;
    prev_voice_active = false;
    // last_active_clock = std::chrono::system_clock::time_point::min();
}

void VAD::stopStream() {
    if (stream != NULL) {
        printf("* Close VAD I/O stream\n");
        // Close
        PaError err = Pa_CloseStream(stream);
        CheckPaError(err);
        stream = NULL;
    }
}

int VAD::paCallback(const void *input_buffer, void *output_buffer,
                    unsigned long frames_per_buffer,
                    const PaStreamCallbackTimeInfo *time_info,
                    PaStreamCallbackFlags status_flags, void *user_data) {
    PaVadData *data = (PaVadData *)user_data;
    int16_t *in = (int16_t *)input_buffer;
    int &len = data->frame_length;

    if (data->enabled && frames_per_buffer >= len) {
        // Scaling
        int16_t buf[len];
        for (int i = 0; i < len; i++) {
            buf[i] = (int16_t)((float)in[i] * data->mic_scale);
        }
        // VAD
        data->activity =
            WebRtcVad_Process(data->vad, data->sampling_rate, buf, len);
    }
    return paContinue;
}
