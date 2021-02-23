#include "pr_pitcam.h"
#include "stb_image_write.h"

#include <fmt/format.h>
#include <cxxopts.hpp>

//-----------------------------------------------------------//
//  This is a command line tool to capture images from the   //
//  high-res pit camera on the top of the pitranger robot.   //
//-----------------------------------------------------------//

int main(int argc, char *argv[]) {
    cxxopts::Options options("pitcap", "Capture Images from PitCam.");

    options.allow_unrecognised_options().add_options()
        ("e,exp", "Image exposure in usec.", cxxopts::value<int>())
        ("i,info", "List camera info.")
        ("hdr", "Capture images across the entire exposure range.")
        ("f,filename", "Image filename (bmp only).", cxxopts::value<std::string>())
        ("positional", "Positional arguments: these are the arguments that are entered without an option", cxxopts::value<std::vector<std::string>>())
        ("h,help", "Print help");

    options.parse_positional({"filename", "positional"});
    auto result = options.parse(argc, argv);

    if (result.count("help") || result.arguments().size() == 0) {
        fmt::print(options.help());
        return 0;
    }

    pr::PitCamera cam;
    int exposure_us = 0;

    if (result.count("info")) {
        fmt::print("Exposure auto: {}\n", cam.get_exposure());
        fmt::print("Exposure min:  {}\n", cam.get_min_exposure());
        fmt::print("Exposure max:  {}\n", cam.get_max_exposure());
        return 0;
    }

    if (result.count("exp")) {
        std::string fname = result["filename"].as<std::string>();
        exposure_us = result["exp"].as<int>();
        fmt::print("Capturing Image\n");
        auto img = cam.capture(exposure_us);
        fmt::print("Saving image {}\n", fname);
        stbi_write_bmp(fname.c_str(), img.cols, img.rows, 3, img.data.data());
        return 0;
    }

    if (result.count("hdr")) {
        int min = cam.get_min_exposure();
        int max = cam.get_max_exposure();

        int e = min;
        while(e < max) {
            fmt::print("Capturing Image With Exposure {}\n", e);
            auto img = cam.capture(e);
            std::string f = fmt::format("{:09}.bmp", e);
            fmt::print("Saving image {}\n", f);
            stbi_write_bmp(f.c_str(), img.cols, img.rows, 3, img.data.data());
            e *= 2;
        }
    }

    return 0;
}
