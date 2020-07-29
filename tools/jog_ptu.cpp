#include "actuators/pr_ptu.h"
#include <cxxopts.hpp>
#include <fmt/format.h>

//-----------------------------------------------------------//
//  This is a command line tool to set the positions of the  //
//  pitranger robot pan/tilt motors for debugging purposes.  //
//-----------------------------------------------------------//

int main(int argc, char *argv[]) {
    pr::PanTiltController ptu;
    cxxopts::Options options("jog_ptu", "Exercise the Pan/Tilt Motors.");

    options.allow_unrecognised_options().add_options()(
        "p,pan", "Pan Degrees", cxxopts::value<int>())("t,tilt", "Tilt Degrees",
                                                       cxxopts::value<int>())(
        "l,list", "List Pan/Tilt Positions")("h,help", "Print help");

    auto result = options.parse(argc, argv);

    if (result.count("pan")) {
        int pan_deg = result["pan"].as<int>();
        fmt::print("Setting Pan to {}\n", pan_deg);
        ptu.set_pan_deg(pan_deg);
    }

    if (result.count("tilt")) {
        int tilt_deg = result["tilt"].as<int>();
        fmt::print("Setting Tilt to {}\n", tilt_deg);
        ptu.set_tilt_deg(tilt_deg);
    }

    if (result.count("list")) {
        auto pan_deg = ptu.get_pan_deg();
        auto tilt_deg = ptu.get_tilt_deg();
        fmt::print("Pan: {} Tilt: {}\n", pan_deg, tilt_deg);
    }

    if (result.count("help") || result.arguments().size() == 0) {
        fmt::print(options.help());
    }

    return 0;
}
