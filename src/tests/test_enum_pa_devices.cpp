#include <stdio.h>
#include <string>
#include <vector>

#include "../modules/audio/audio.h"
#include "../modules/audio/vad.h"

int main(int argc, char const *argv[]) {
    InitializePortAudio();

    printf("* Enumerate PortAudio devices\n");
    std::vector<const PaDeviceInfo *> devices;
    EnumeratePortAudioDevices(devices);
    for (int i = 0; i < devices.size(); i++) {
        printf(" %d: [in: %d / out: %d]  \"%s\"\n", i,
               devices[i]->maxInputChannels, devices[i]->maxOutputChannels,
               devices[i]->name);
    }

    TerminatePortAudio();
    return 0;
}
