#include "actuators/pr_wheel.h"
#include "pr_time.h"
#include <cxxopts.hpp>
#include <fmt/format.h>

//-----------------------------------------------------------//
//  This is a command line tool to set the velocities of the //
//  pitranger robot drive motors for debugging purposes.     //
//-----------------------------------------------------------//

int main(int argc, char *argv[]) {
    pr::WheelController wheels;
    cxxopts::Options options("jog_wheels", "Exercise the Wheel Motors.");

    options.allow_unrecognised_options().add_options()(
        "g,go", "Set all Wheel RPMs", cxxopts::value<double>())(
        "fr", "Front-Right Wheel RPM", cxxopts::value<double>())(
        "fl", "Front-Left Wheel RPM", cxxopts::value<double>())(
        "rr", "Rear-Right Wheel RPM", cxxopts::value<double>())(
        "rl", "Rear-Left Wheel RPM", cxxopts::value<double>())(
        "l,list", "List Wheel Velocities")("h,help", "Print help");

    auto result = options.parse(argc, argv);

    double fr_rpm = wheels.get_front_right_rpm();
    double fl_rpm = wheels.get_front_left_rpm();
    double rr_rpm = wheels.get_rear_right_rpm();
    double rl_rpm = wheels.get_rear_left_rpm();

    if (result.count("help") || result.arguments().size() == 0) {
        fmt::print(options.help());
        return 0;
    }

    if (result.count("list")) {
        fmt::print("Current Motor Speeds [RPM]:\n");
        fmt::print("{:6.3f}\t{:6.3f}\n{:6.3f}\t{:6.3f}\n\n", fl_rpm, fr_rpm,
                   rl_rpm, rr_rpm);
        return 0;
    }
    if (result.count("go")) {
        fr_rpm = result["go"].as<double>();
        fl_rpm = result["go"].as<double>();
        rr_rpm = result["go"].as<double>();
        rl_rpm = result["go"].as<double>();
    }

    if (result.count("fr")) {
        fr_rpm = result["fr"].as<double>();
    }
    if (result.count("fl")) {
        fl_rpm = result["fl"].as<double>();
    }
    if (result.count("rr")) {
        rr_rpm = result["rr"].as<double>();
    }
    if (result.count("rl")) {
        rl_rpm = result["rl"].as<double>();
    }

    fr_rpm = wheels.set_front_right_rpm(fr_rpm);
    fl_rpm = wheels.set_front_left_rpm(fl_rpm);
    rr_rpm = wheels.set_rear_right_rpm(rr_rpm);
    rl_rpm = wheels.set_rear_left_rpm(rl_rpm);

    fmt::print("New Motor Speeds [RPM]:\n");
    fmt::print("{:8.3f}\t{:8.3f}\n{:8.3f}\t{:8.3f}\n\n", fl_rpm, fr_rpm, rl_rpm,
               rr_rpm);

    return 0;
}
