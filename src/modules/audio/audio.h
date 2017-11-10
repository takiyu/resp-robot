#ifndef AUDIO_160625
#define AUDIO_160625

#include <portaudio.h>
#include <stdio.h>
#include <string>
#include <vector>

#include "../../config.h"

void CheckPaError(PaError err);

void InitializePortAudio();

void TerminatePortAudio();

void EnumeratePortAudioDevices(std::vector<const PaDeviceInfo *> &dev_names);

int GetPortAudioDeviceId(bool input);

void GetPortAudioStreamParams(PaStreamParameters &params, int n_channels,
                              PaSampleFormat format, double sampling_rate,
                              bool input);

class WavPlayer {
public:
    WavPlayer() : stream(NULL) {}
    ~WavPlayer() { stop(); }

    void play(FILE *wavfp, bool autoclose_fp, bool block);
    void stop();
    bool isPlaying() { return Pa_IsStreamActive(stream) > 0; }

private:
    struct PaWavData {
        FILE *wavfp;
        int bits_width;
        int n_channels;
        bool autoclose_fp;
    };

    struct PaWavData data;
    PaStream *stream;

    static int paCallback(const void *input_buffer, void *output_buffer,
                          unsigned long frames_per_buffer,
                          const PaStreamCallbackTimeInfo *time_info,
                          PaStreamCallbackFlags status_flags, void *user_data);
};

#endif /* end of include guard */
