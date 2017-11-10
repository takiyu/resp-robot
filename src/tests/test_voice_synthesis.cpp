#include <stdio.h>
#include <unistd.h>

#include "../modules/audio/audio.h"
#include "../modules/audio/voice_synthesis.h"

int main(int argc, char const* argv[]) {
    InitializePortAudio();

    VoiceSynthesis voice_synthesis;
    voice_synthesis.init();

    WavPlayer player;

    // Simple version
    voice_synthesis.synthesis("テストテストテストテスト", player, false);
    sleep(1);
    // intercept
    player.stop();

    // Blocking
    voice_synthesis.synthesis("こんにちは", player, true);
    voice_synthesis.synthesis("あいうえお", player, true);

    // File pointer version
    FILE* file = tmpfile();
    voice_synthesis.synthesis("ばいばい", file);
    fseek(file, 0, SEEK_SET);
    player.play(file, false, true);
    fclose(file);

    TerminatePortAudio();
    return 0;
}
