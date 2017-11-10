#include "audio.h"

#include <string.h>
#include <unistd.h>
#include <limits>

namespace {
int INPUT_DEV_ID = -1, OUTPUT_DEV_ID = -1;

inline int FindNearestString(const std::vector<std::string> &candidates,
                             const std::string &key) {
    // Find nearest candidate
    int min_len = std::numeric_limits<int>::max();
    int min_idx = -1;
    for (int i = 0; i < candidates.size(); i++) {
        if (candidates[i].find(key) != std::string::npos &&
            candidates[i].size() < min_len) {
            min_idx = i;
            min_len = candidates[i].size();
        }
    }
    return min_idx;
}

void SetupPortAudioDeviceId(bool input) {
    std::vector<const PaDeviceInfo *> devices;
    EnumeratePortAudioDevices(devices);

    // Find key matched device
    const std::string &KEY = input ? PA_INPUT_DEV_KEY : PA_OUTPUT_DEV_KEY;
    std::vector<std::string> dev_names;
    for (int i = 0; i < devices.size(); i++) {
        dev_names.push_back(std::string(devices[i]->name));
    }
    int id = FindNearestString(dev_names, KEY);

    if (id < 0) {
        // Find valid device
        int max_cnt = 0;
        for (int i = 0; i < devices.size(); i++) {
            int n_channels;
            if (input) {
                n_channels = devices[i]->maxInputChannels;
            } else {
                n_channels = devices[i]->maxOutputChannels;
            }
            if (n_channels > max_cnt) {
                id = i;
                max_cnt = n_channels;
            }
        }
    }

    printf(" >> %s: [%d] \"%s\"\n", input ? "input" : "output", id,
           dev_names[id].c_str());
    if (input) {
        INPUT_DEV_ID = id;
    } else {
        OUTPUT_DEV_ID = id;
    }
}

inline std::string FreadGetStr(FILE *fp, int len) {
    char buf[len + 1];
    if (fread(buf, sizeof(char), len, fp) != len) {
        printf(" >> Invalid file format (Get string)\n");
        return std::string();
    }
    buf[len] = '\0';
    return std::string(buf);
}

inline bool freadCheckStr(FILE *fp, const std::string &s) {
    return (FreadGetStr(fp, s.size()) == s);
}

template <typename T>
inline T freadGetNum(FILE *fp) {
    int size = sizeof(T);
    unsigned char buf[size];
    if (fread(&buf, size, 1, fp) != 1) {
        printf(" >> Invalid file format (Get number)\n");
        return 0;
    }
    T ret = 0;
    for (int i = 0; i < size; i++) {
        ret |= (buf[i] << (8 * i));  // for machine endian
    }
    return ret;
}

}  // namespace

void CheckPaError(PaError err) {
    if (err != paNoError) {
        printf("* PortAudio Error: %s\n", Pa_GetErrorText(err));
    }
}

void InitializePortAudio() {
    printf("* Initialize PortAudio\n");
    PaError err = Pa_Initialize();
    CheckPaError(err);

    printf("* Setup PortAudio device ids\n");
    SetupPortAudioDeviceId(true);
    SetupPortAudioDeviceId(false);
}

void TerminatePortAudio() {
    printf("* Terminate PortAudio\n");
    PaError err = Pa_Terminate();
    CheckPaError(err);
}

void EnumeratePortAudioDevices(std::vector<const PaDeviceInfo *> &dev_names) {
    dev_names.clear();
    int n_dev = Pa_GetDeviceCount();
    if (n_dev < 0) {
        printf("* Failed to enumerate PortAudio devices\n");
        return;
    }
    const PaDeviceInfo *dev_info;
    for (int i = 0; i < n_dev; i++) {
        dev_info = Pa_GetDeviceInfo(i);
        dev_names.push_back(dev_info);
    }
}

int GetPortAudioDeviceId(bool input) {
    return input ? INPUT_DEV_ID : OUTPUT_DEV_ID;
}

void GetPortAudioStreamParams(PaStreamParameters &params, int n_channels,
                              PaSampleFormat format, double sampling_rate,
                              bool input) {
    int dev_id = GetPortAudioDeviceId(input);
    // Set
    params.channelCount = n_channels;
    params.device = dev_id;
    params.hostApiSpecificStreamInfo = NULL;
    params.sampleFormat = format;
    params.suggestedLatency = Pa_GetDeviceInfo(dev_id)->defaultLowInputLatency;
    params.hostApiSpecificStreamInfo = NULL;

    // Check
    PaError err;
    if (input) {
        err = Pa_IsFormatSupported(&params, NULL, sampling_rate);
    } else {
        err = Pa_IsFormatSupported(NULL, &params, sampling_rate);
    }
    if (err != paFormatIsSupported) {
        printf("* Invalid PortAudio %s parameters\n",
               input ? "input" : "output");
    }
}

void WavPlayer::play(FILE *wavfp, bool autoclose_fp, bool block) {
    stop();  // Stop previous play

    printf("* Play WAVE file\n");
    bool has_fmt = false;
    uint16_t format;
    uint16_t n_channels;
    uint32_t sampling_rate;
    uint16_t bits_width;
    PaSampleFormat paFormat;

    // Read WAVE file
    if (!freadCheckStr(wavfp, "RIFF")) return;
    uint32_t total_size = freadGetNum<uint32_t>(wavfp);
    if (!freadCheckStr(wavfp, "WAVE")) return;
    while (true) {
        std::string chunk = FreadGetStr(wavfp, 4);
        uint32_t chunk_len = freadGetNum<uint32_t>(wavfp);
        if (chunk == "fmt ") {
            // Read format
            format = freadGetNum<uint16_t>(wavfp);
            n_channels = freadGetNum<uint16_t>(wavfp);
            sampling_rate = freadGetNum<uint32_t>(wavfp);
            uint32_t byte_par_sec = freadGetNum<uint32_t>(wavfp);
            uint16_t block_align = freadGetNum<uint16_t>(wavfp);
            bits_width = freadGetNum<uint16_t>(wavfp);
            // Check
            if (byte_par_sec != sampling_rate * n_channels * bits_width / 8 ||
                block_align != n_channels * bits_width / 8) {
                printf(" >> Invalid file format (Read format)\n");
                return;
            }
            // Skip extended
            int left_size = chunk_len - 16;
            if (left_size > 0) {
                if (fseek(wavfp, left_size, SEEK_CUR) != 0) {
                    printf(" >> Invalid file format (Skip extended)\n");
                    return;
                }
            }
        } else if (chunk == "data") {
            float sec = float(chunk_len) / float(sampling_rate) /
                        float(n_channels) / float(bits_width / 8);
            printf(" >> WAVE length : %.0f sec\n", sec);
            break;
        } else {
            // skip chunk
            if (fseek(wavfp, chunk_len, SEEK_CUR) != 0) {
                printf(" >> Invalid file format (Skip chunk)\n");
                return;
            }
        }
    }

    // Format
    if (format == 1) {
        if (bits_width == 8)
            paFormat = paInt8;
        else if (bits_width == 16)
            paFormat = paInt16;
        else if (bits_width == 32)
            paFormat = paInt32;
        else {
            printf(" >> Invalid bit width: %u\n", bits_width);
            return;
        }
    } else {
        printf(" >> Unknown format: %u\n", format);
        return;
    }

    // Set data
    data.wavfp = wavfp;
    data.bits_width = bits_width;
    data.n_channels = n_channels;
    data.autoclose_fp = autoclose_fp;

    // Get device parameters
    PaStreamParameters output_params;
    GetPortAudioStreamParams(output_params, (int)n_channels, paFormat,
                             (double)sampling_rate, false);
    // Open
    PaError err = Pa_OpenStream(&stream, NULL, &output_params,
                                (double)sampling_rate, PA_FRAMES_PER_BUFFER,
                                paNoFlag, WavPlayer::paCallback, &data);
    CheckPaError(err);

    // Start
    err = Pa_StartStream(stream);
    CheckPaError(err);

    if (block) {
        while (isPlaying()) usleep(10000);
    }
}

void WavPlayer::stop() {
    if (stream != NULL) {
        printf("* Stop to play WAVE\n");
        if (Pa_IsStreamActive(stream) > 0) {
            PaError err1 = Pa_StopStream(stream);
            CheckPaError(err1);
        }
        // Close wave file
        if (data.autoclose_fp) {
            fclose(data.wavfp);
        }
        // Close
        PaError err2 = Pa_CloseStream(stream);
        stream = NULL;
        CheckPaError(err2);
    }
}

int WavPlayer::paCallback(const void *input_buffer, void *output_buffer,
                          unsigned long frames_per_buffer,
                          const PaStreamCallbackTimeInfo *time_info,
                          PaStreamCallbackFlags status_flags, void *user_data) {
    PaWavData *data = (PaWavData *)user_data;
    uint8_t *out = (uint8_t *)output_buffer;

    size_t len = fread(out, data->bits_width * data->n_channels / 8,
                       frames_per_buffer, data->wavfp);
    out += len * data->bits_width * data->n_channels / 8;

    int left_size = frames_per_buffer - len;
    if (left_size) {
        for (int i = 0; i < left_size; i++) out[i] = 0;
        printf("* Finished to play WAVE\n");
        return paComplete;
    }

    return paContinue;
}
