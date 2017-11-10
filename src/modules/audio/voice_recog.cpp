#include "voice_recog.h"

#include <sstream>

#include "imgui/imgui.h"
#include "julius/juliuslib.h"

namespace {
// Callbacks
void ReadyCallback(Recog *recog, void *user_data) {
    VoiceRecognition::RecogData *data =
        (VoiceRecognition::RecogData *)user_data;
}

void StartCallback(Recog *recog, void *user_data) {
    VoiceRecognition::RecogData *data =
        (VoiceRecognition::RecogData *)user_data;
    data->processing = true;
}

void ResultCallback(Recog *recog, void *user_data) {
    VoiceRecognition::RecogData *data =
        (VoiceRecognition::RecogData *)user_data;

    std::stringstream result_ss;
    for (const RecogProcess *r = recog->process_list; r; r = r->next) {
        WORD_INFO *winfo = r->lm->winfo;
        for (int n = 0; n < r->result.sentnum; ++n) {
            Sentence *s = &(r->result.sent[n]);
            WORD_ID *seq = s->word;
            if (seq == NULL) continue;
            for (int i = 0; i < s->word_num; ++i) {
                result_ss << winfo->woutput[seq[i]];
            }
        }
    }

    if (!result_ss.str().empty()) {
        data->results.push(result_ss.str());
    }

    data->processing = false;
}
}  // namespace

class VoiceRecognition::Impl {
public:
    Impl() : recog(NULL) {}
    ~Impl() {}
    Recog *recog;
};

VoiceRecognition::VoiceRecognition(int cache_size)
    : recog_thread(NULL), CACHE_SIZE(cache_size) {
    impl = new Impl;
}

VoiceRecognition::~VoiceRecognition() {
    exit();
    delete impl;
}

bool VoiceRecognition::init() {
    printf("* Initialize speech recognition\n");
    printf(" >> Julius %s\n", JULIUS_VERSION);

    // Disable logging
    jlog_set_output(NULL);

    // Load config files
    Jconf *jconf = j_jconf_new();
    for (int i = 0; i < N_JULIUS_CONFIG; i++) {
        const char *CONFIG = JULIUS_CONFIGS[i].c_str();
        int ret = j_config_load_file(jconf, const_cast<char *>(CONFIG));
        if (ret != 0) {
            printf(" >> Invalid config file (%s)\n", CONFIG);
            // Release config
            j_jconf_free(jconf);
            return false;
        }
    }
    j_jconf_finalize(jconf);

    // Create Julius instance
    this->impl->recog = j_create_instance_from_jconf(jconf);
    if (impl->recog == NULL) {
        printf("* Failed to create Julius instance\n");
        // Release config
        j_jconf_free(jconf);
        return false;
    }

    // Regster callbacks
    callback_add(impl->recog, CALLBACK_EVENT_SPEECH_READY, ReadyCallback,
                 &data);
    callback_add(impl->recog, CALLBACK_EVENT_SPEECH_START, StartCallback,
                 &data);
    callback_add(impl->recog, CALLBACK_RESULT, ResultCallback, &data);

    // Initialize audio input
    if (j_adin_init(impl->recog) == FALSE) {
        printf("* Failed to initialize A/D device\n");
        return false;
    }

    // 	// output system information to log
    // 	j_recog_info(impl->recog);

    startStream();

    // Cache array
    processing_cache.resize(CACHE_SIZE, 0);
    processing_cache_idx = 0;

    return true;
}

void VoiceRecognition::exit() {
    if (impl->recog != NULL) {
        // stop
        enable();
        stopStream();
        // release
        printf("* Release Julius instance\n");
        // if (impl->recog->jconf != NULL) {
        //     j_jconf_free(impl->recog->jconf); // segmentation fault
        // }
        j_recog_free(impl->recog);
        impl->recog = NULL;
    }
}

void VoiceRecognition::disable() {
    if (impl->recog != NULL && enabled) {
        enabled = false;
        j_request_pause(impl->recog);
    }
}

std::string VoiceRecognition::popResult() {
    if (data.results.empty()) {
        return "";
    } else {
        std::string res = data.results.front();
        data.results.pop();
        poped_result = res;  // cache for drawing
        return res;
    }
}

void VoiceRecognition::drawGlUi() {
    // Update cache
    processing_cache[processing_cache_idx] =
        static_cast<float>(data.processing);
    processing_cache_idx = (processing_cache_idx + 1) % CACHE_SIZE;

    // Common Window
    ImGui::Begin("Modules");
    // Header
    if (ImGui::CollapsingHeader("Voice Recognition",
                                ImGuiTreeNodeFlags_DefaultOpen)) {
        // Is processing
        ImGui::Text(data.processing ? "Processing" : "Not Processing");
        // Last result
        std::string result;
        if (data.results.empty()) {
            result = poped_result;
        } else {
            result = data.results.back();
        }
        ImGui::Text("%s", ("Last Result: " + result).c_str());
        // Processing
        ImGui::PlotLines("Processing", &processing_cache[0], CACHE_SIZE,
                         processing_cache_idx, NULL, 0.0f, 1.0f, ImVec2(0, 40));
    }
    ImGui::End();
}

void VoiceRecognition::startStream() {
    printf("* Start speech recognition\n");
    // Set initial value
    this->data.processing = false;
    this->enabled = true;

    // Open input stream and recognize
    std::stringstream dev_id;
    dev_id << GetPortAudioDeviceId(true);
    switch (
        j_open_stream(impl->recog, const_cast<char *>(dev_id.str().c_str()))) {
        case 0:
            break;  // success
        case -1: printf(" >> Error in input stream"); return;
        case -2: printf(" >> Failed to begin input stream"); return;
    }

    // Start recognition thread
    recog_thread = new boost::thread([&]() {
        j_recognize_stream(impl->recog);
        printf("* Stop speech recognition thread\n");
    });
}

void VoiceRecognition::stopStream() {
    if (impl->recog != NULL && recog_thread != NULL) {
        printf("* Close Julius stream\n");
        j_close_stream(impl->recog);
        recog_thread->join();
        delete recog_thread;
        recog_thread = NULL;
    }
}

void VoiceRecognition::enable() {
    if (impl->recog != NULL && !enabled) {
        enabled = true;
        j_request_resume(impl->recog);
    }
}
