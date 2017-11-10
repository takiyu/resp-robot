#ifndef VOICE_SYNTHESIS_160625
#define VOICE_SYNTHESIS_160625

#include <stdio.h>
#include <string.h>
#include <string>

#include "HTS_engine.h"
#include "jpcommon/jpcommon.h"
#include "mecab/src/mecab.h"
#include "njd/njd.h"

#include "audio.h"

#include "../../config.h"

class VoiceSynthesis {
public:
    VoiceSynthesis() : jtalk(NULL) {}
    ~VoiceSynthesis() { exit(); }

    bool init();
    void exit();

    bool synthesis(const std::string &text, FILE *wavfp);
    bool synthesis(const std::string &text, WavPlayer &player,
                   bool block = false);

    // Voice parameters
    void set_sampling_frequency(size_t i = 48000);
    void set_fperiod(size_t i = 240);
    void set_alpha(double f = 0.5);
    void set_beta(double f = 0.2);
    void set_speed(double f = 1.0);
    void add_half_tone(double f = 0.0);
    void set_msd_threshold(size_t i = 1, double f = 0.5);
    void set_gv_weight(size_t i = 0, double f = 1.0);
    void set_volume(double f);
    void set_audio_buff_size(size_t i);

    // GL UI
    void drawGlUi();

private:
    // OpenJTalk instance
    struct OpenJTalkInst {
        Mecab mecab;
        NJD njd;
        JPCommon jpcommon;
        HTS_Engine engine;
    };
    struct OpenJTalkInst *jtalk;

    // Cache for drawing
    std::string synthesised_text;
};

#endif /* end of include guard */
