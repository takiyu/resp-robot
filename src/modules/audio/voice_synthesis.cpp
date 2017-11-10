#include "voice_synthesis.h"

#include "mecab2njd/mecab2njd.h"
#include "njd2jpcommon/njd2jpcommon.h"
#include "njd_set_accent_phrase/njd_set_accent_phrase.h"
#include "njd_set_accent_type/njd_set_accent_type.h"
#include "njd_set_digit/njd_set_digit.h"
#include "njd_set_long_vowel/njd_set_long_vowel.h"
#include "njd_set_pronunciation/njd_set_pronunciation.h"
#include "njd_set_unvoiced_vowel/njd_set_unvoiced_vowel.h"
#include "text2mecab/text2mecab.h"

#define MAXBUFLEN 1024  // This macro is also defined in open_jtalk sources.

#include "imgui/imgui.h"

void VoiceSynthesis::exit() {
    if (jtalk != NULL) {
        printf("* Release OpenJTalk instance\n");
        Mecab_clear(&jtalk->mecab);
        NJD_clear(&jtalk->njd);
        JPCommon_clear(&jtalk->jpcommon);
        HTS_Engine_clear(&jtalk->engine);
        delete jtalk;
        jtalk = NULL;
    }
}

bool VoiceSynthesis::init() {
    printf("* Initialize voice synthesis\n");
    printf(" >> OpenJTalk %s\n", "1.09");

    // Initialize
    jtalk = new struct OpenJTalkInst();
    Mecab_initialize(&jtalk->mecab);
    NJD_initialize(&jtalk->njd);
    JPCommon_initialize(&jtalk->jpcommon);
    HTS_Engine_initialize(&jtalk->engine);

    // Load dictionary and HTS voice
    if (Mecab_load(&jtalk->mecab, OPEN_JTALK_DICT.c_str()) != TRUE) {
        printf(" >> Failed to load dictionary (%s)\n", OPEN_JTALK_DICT.c_str());
        return false;
    }
    const char *HTS_VOICE = OPEN_JTALK_HTS_VOICE.c_str();
    if (HTS_Engine_load(&jtalk->engine, const_cast<char **>(&HTS_VOICE), 1) !=
        TRUE) {
        printf(" >> Failed to load hts voice (%s)\n",
               OPEN_JTALK_HTS_VOICE.c_str());
        return false;
    }
    if (strcmp(HTS_Engine_get_fullcontext_label_format(&jtalk->engine),
               "HTS_TTS_JPN") != 0) {
        printf(" >> Failed to initialize HTS\n");
        return false;
    }

    // Set default voice parameters
    set_sampling_frequency(48000);
    set_fperiod(230);
    add_half_tone(2.0);

    return true;
}

bool VoiceSynthesis::synthesis(const std::string &text, FILE *wavfp) {
    printf("* Voice Synthesis: %s\n", text.c_str());
    bool res = false;
    char buff[MAXBUFLEN];

    text2mecab(buff, text.c_str());
    Mecab_analysis(&jtalk->mecab, buff);
    mecab2njd(&jtalk->njd, Mecab_get_feature(&jtalk->mecab),
              Mecab_get_size(&jtalk->mecab));
    njd_set_pronunciation(&jtalk->njd);
    njd_set_digit(&jtalk->njd);
    njd_set_accent_phrase(&jtalk->njd);
    njd_set_accent_type(&jtalk->njd);
    njd_set_unvoiced_vowel(&jtalk->njd);
    njd_set_long_vowel(&jtalk->njd);
    njd2jpcommon(&jtalk->jpcommon, &jtalk->njd);
    JPCommon_make_label(&jtalk->jpcommon);
    if (JPCommon_get_label_size(&jtalk->jpcommon) > 2) {
        if (HTS_Engine_synthesize_from_strings(
                &jtalk->engine, JPCommon_get_label_feature(&jtalk->jpcommon),
                JPCommon_get_label_size(&jtalk->jpcommon)) == TRUE) {
            res = true;
        }
        if (wavfp != NULL) {
            HTS_Engine_save_riff(&jtalk->engine, wavfp);
        }

#if 0
        // Log output
        FILE *logfp = stdout;
        fprintf(logfp, "[Text analysis result]\n");
        NJD_fprint(&jtalk->njd, logfp);
        fprintf(logfp, "\n[Output label]\n");
        HTS_Engine_save_label(&jtalk->engine, logfp);
        fprintf(logfp, "\n");
        HTS_Engine_save_information(&jtalk->engine, logfp);
#endif

        HTS_Engine_refresh(&jtalk->engine);
    }

    JPCommon_refresh(&jtalk->jpcommon);
    NJD_refresh(&jtalk->njd);
    Mecab_refresh(&jtalk->mecab);

    if (!res) {
        printf(" >> Failed to synthesize: %s\n", text.c_str());
    } else {
        synthesised_text = text;
    }
    return res;
}

bool VoiceSynthesis::synthesis(const std::string &text, WavPlayer &player,
                               bool block) {
    if (text.empty()) {
        player.stop();
    } else {
        FILE *wavfp = tmpfile();
        bool ret = synthesis(text, wavfp);
        if (!ret) return false;
        fseek(wavfp, 0, SEEK_SET);
        player.play(wavfp, true, block);
    }
    return true;
}

void VoiceSynthesis::set_sampling_frequency(size_t i) {
    HTS_Engine_set_sampling_frequency(&jtalk->engine, i);
}

void VoiceSynthesis::set_fperiod(size_t i) {
    HTS_Engine_set_fperiod(&jtalk->engine, i);
}

void VoiceSynthesis::set_alpha(double f) {
    HTS_Engine_set_alpha(&jtalk->engine, f);
}

void VoiceSynthesis::set_beta(double f) {
    HTS_Engine_set_beta(&jtalk->engine, f);
}

void VoiceSynthesis::set_speed(double f) {
    HTS_Engine_set_speed(&jtalk->engine, f);
}

void VoiceSynthesis::add_half_tone(double f) {
    HTS_Engine_add_half_tone(&jtalk->engine, f);
}

void VoiceSynthesis::set_msd_threshold(size_t i, double f) {
    HTS_Engine_set_msd_threshold(&jtalk->engine, i, f);
}

void VoiceSynthesis::set_gv_weight(size_t i, double f) {
    HTS_Engine_set_gv_weight(&jtalk->engine, i, f);
}

void VoiceSynthesis::set_volume(double f) {
    HTS_Engine_set_volume(&jtalk->engine, f);
}

void VoiceSynthesis::set_audio_buff_size(size_t i) {
    HTS_Engine_set_audio_buff_size(&jtalk->engine, i);
}

void VoiceSynthesis::drawGlUi() {
    // Common Window
    ImGui::Begin("Modules");
    // Header
    if (ImGui::CollapsingHeader("Voice Synthesis",
                                ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("Last Text: %s", synthesised_text.c_str());
    }
    ImGui::End();
}
