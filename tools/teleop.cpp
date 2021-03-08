#include "pitranger/pitranger.h"
#include "actuators/pr_ptu.h"
#include "actuators/pr_wheel.h"
#include <steam_controller/steam_controller.hpp>
#include "pr_imu.h"
#include "pr_log.h"
#include "pr_time.h"
#include <fmt/format.h>
#include <fmt/ostream.h>

namespace sc = steam_controller;

sc::connection_state check_status(const std::unique_ptr<sc::controller>& controller) {
    sc::event event{};
    controller->poll(event);
    return controller->state();
}

std::unique_ptr<sc::controller> find_steam_controller() {
    sc::context ctx;
    std::vector<sc::connection_info> conn_infos = ctx.enumerate();

    // Enumerate connected controllers.
    while ( conn_infos.empty() ) {
        fmt::print("Failed to connect to steam controller. Is the dongle plugged in? Retrying...\n");
        conn_infos = ctx.enumerate();
        pr::time::sleep(1);
    }

    // Find the first connected controller.
    std::unique_ptr<sc::controller> controller;
    for(auto ci : conn_infos) {

        controller = ctx.connect(ci, 0, std::chrono::milliseconds(500));
        if( !controller ) { continue; }

        auto state = check_status(controller);
        if( state == sc::connection_state::connecting ) {
            //fmt::print("Controller connecting.\n");
            while( state == sc::connection_state::connecting ) {
                state = check_status(controller);
            }
        }
        if( state == sc::connection_state::disconnected ) {
            //fmt::print("Controller disconnected.\n");
        } else if( state == sc::connection_state::connected ) {
            //fmt::print("Controller connected.\n");
            return controller;
        } else {
            fmt::print("Controller status {} not recognized.\n", state);
        }
    }
    return {};
}

int main(int argc, char *argv[]) {
    std::unique_ptr<sc::controller> ctrl = find_steam_controller();
    if( !ctrl ) { return -1; }

    pr::WheelController motor_controller;
    pr::PanTiltController ptu;

    fmt::print("All devices connected!\n");

    motor_controller.set_left_rpm(0);
    motor_controller.set_right_rpm(0);

    sc::event last_event {};
    sc::event event {};

    const double pan = ptu.get_pan_deg();
    const double tilt = ptu.get_tilt_deg();

    const int N_PANS = 5;
    const int N_TILTS = 12;
    const std::array<double, N_PANS> pans = {-90, -45, 0, 45, 90};
    const std::array<double, N_TILTS> tilts= {-70, -60, -50, -40, -30, -20, -10, 0, 30, 45, 60, 90};

    int pan_idx = std::distance(pans.begin(), std::lower_bound(pans.begin(), pans.end(), pan));
    int tilt_idx = std::distance(tilts.begin(), std::lower_bound(tilts.begin(), tilts.end(), tilt));

    while(1) {
        ctrl->poll(event);
        auto state = ctrl->state();
        for(int i=0; i<100; ++i) {
            ctrl->poll(event);
            state = ctrl->state();
        }

        // Stop motors if controller connection is lost.
        if(state == sc::connection_state::disconnected) {
            fmt::print("Lost connection to steam controller.\n");

            motor_controller.set_left_rpm(0);
            motor_controller.set_right_rpm(0);
        }

        if( event.key == sc::event_key::UPDATE ) {
            const auto& buttons = event.update.buttons;
            const auto& last_buttons = last_event.update.buttons;

            if( (event.update.buttons & 0x20) && !(last_event.update.buttons & 0x20)) {
                pan_idx = std::min<int>(pan_idx+1, N_PANS-1);
            }
            if( (event.update.buttons & 0x40) && !(last_event.update.buttons & 0x40)) {
                pan_idx = std::max<int>(pan_idx-1, 0);
            }
            if( (event.update.buttons & 0x80) && !(last_event.update.buttons & 0x80)) {
                tilt_idx = std::min<int>(tilt_idx+1, N_TILTS-1);
            }
            if( (event.update.buttons & 0x10) && !(last_event.update.buttons & 0x10)) {
                tilt_idx = std::max<int>(tilt_idx-1, 0);
            }
            if( (event.update.buttons & 0x4000) && !(last_event.update.buttons & 0x4000)) {
                for(int i=7; i>=0; --i) {

                    try {
                        ptu.set_tilt_deg(tilts[i]);
                    } catch( const std::runtime_error& e) {}

                    if(i%2) {
                        for(int j=0; j<N_PANS; ++j) {
                            try {
                                ptu.set_pan_deg(pans[j]);
                            } catch( const std::runtime_error& e) {}
                            pr::time::sleep(1);
                        }
                    } else {
                        for(int j=N_PANS-1; j>=0; --j) {
                            ptu.set_pan_deg(pans[j]);
                            pr::time::sleep(1);
                        }
                    }
                }
            }
            try {
                ptu.set_pan_deg(pans[pan_idx]);
                ptu.set_tilt_deg(tilts[tilt_idx]);
            } catch( const std::runtime_error& e) {}

            const double x = -1 * event.update.left_axis.x / static_cast<double>(1<<15);
            const double y = event.update.left_axis.y / static_cast<double>(1<<15);
            const double u = (1.0-std::abs(x)) * y + y;
            const double v = (1.0-std::abs(y)) * x + x;
            const double l = (u-v)/2.0;
            const double r = (u+v)/2.0;
            
            double speed = 12.0;
            if (event.update.buttons & 0x2) {
                speed *= 2.0;
            }

            double  left_rpm = l * speed;
            double right_rpm = r * speed;

            try {
                motor_controller.set_left_rpm(left_rpm);
                motor_controller.set_right_rpm(right_rpm);
            } catch( const std::runtime_error& e ) {
                fmt::print("Ignoring exception {}\n", e.what());
            }
        }

        last_event = event;
        pr::time::msleep(10);
    }

    return 0;
}
