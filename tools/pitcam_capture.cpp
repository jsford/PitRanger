#include "pr_pitcam.h"

int main(int argc, char** argv) {

    pr::PitCamera pitcam;

    for(int i=0; i<10; ++i) {
        auto img = pitcam.capture(1000);
    }

    return 0;
}
