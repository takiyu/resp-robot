#ifndef SPEECH_RECOG_160624
#define SPEECH_RECOG_160624

#include <boost/thread.hpp>
#include <queue>
#include <string>

#include "audio.h"

#include "../../config.h"

class VoiceRecognition {
public:
    VoiceRecognition(int cache_size = 50);
    ~VoiceRecognition();

    bool init();
    void exit();

    void enable();
    void disable();

    bool isProcessing() { return data.processing; }
    bool hasResult() { return !data.results.empty(); }
    std::string popResult();

    // GL UI
    void drawGlUi();

    struct RecogData {
        bool processing;
        std::queue<std::string> results;
    };

private:
    class Impl;
    Impl *impl;

    bool enabled;

    boost::thread *recog_thread;
    struct RecogData data;

    void startStream();
    void stopStream();

    // Cache for drawing
    std::string poped_result;
    const int CACHE_SIZE;
    std::vector<float> processing_cache;
    int processing_cache_idx;
};

#endif
