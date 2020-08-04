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
        ("f,filename", "Image filename (only jpg).", cxxopts::value<std::string>())
        ("positional", "Positional arguments: these are the arguments that are entered without an option", cxxopts::value<std::vector<std::string>>())
        ("h,help", "Print help");

    options.parse_positional({"filename", "positional"});
    auto result = options.parse(argc, argv);

    if (result.count("help") || result.arguments().size() == 0) {
        fmt::print(options.help());
        return 0;
    }

    pr::PitCamera cam;

    if (result.count("exp")) {
        int exp = result["exp"].as<int>();
        fmt::print("Exposure not implemented yet.");
        //cam.setExposure(exp);
    }

    std::string fname = result["filename"].as<std::string>();
    fmt::print("Capturing Image\n");
    auto img = cam.capture(1000);
    fmt::print("Saving image {}\n", fname);
    stbi_write_jpg(fname.c_str(), img.cols, img.rows, 3, img.data.data(), 85);

    return 0;
}
