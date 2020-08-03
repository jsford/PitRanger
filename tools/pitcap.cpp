#include "pr_pitcam.h"
#include "stb_image_write.h"

#include <fmt/format.h>

int main(int argc, char** argv) {

    pr::PitCamera pitcam;

    for(int i=0; i<100; ++i) {
        auto img = pitcam.capture(1000);

        const std::string fname = fmt::format("{}.jpg", i);
        fmt::print("Saving image {}\n", fname);
        stbi_write_jpg(fname.c_str(), img.cols, img.rows, 3, img.data.data(), 85);
    }

    return 0;
}
