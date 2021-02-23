#include "pitranger/pitranger.h"
#include "actuators/pr_wheel.h"

#include <stdio.h>
#include <algorithm>
#include <vector>
#include <ncurses.h>
#include <string>
#include <fstream>
#include <unistd.h>
#include<sys/time.h>

#define WIDTH 30
#define HEIGHT 10 

#define ROVER_WIDTH 0.62
#define WHEEL_DIAM 0.2032
#define MAX_SPEED (24*M_PI*WHEEL_DIAM/60.0)
#define RPM2MPS(rpm) ((rpm)*M_PI*WHEEL_DIAM/60.0)

int startx = 0;
int starty = 0;

long long time_in_ms(void) {
    struct timeval tv;

    gettimeofday(&tv,NULL);
    return (((long long)tv.tv_sec)*1000)+(tv.tv_usec/1000);
}

void print_menu(WINDOW *menu_win, double speed_rpm, double radius, bool go)
{
	int x, y, i;	

	x = 2;
	y = 2;
	box(menu_win, 0, 0);
    if( speed_rpm < 0.0 ) {
        mvwprintw(menu_win, y++, x, "speed [rpm]: %4.1f  ", speed_rpm);
    } else {
        mvwprintw(menu_win, y++, x, "speed [rpm]:  %4.1f  ", speed_rpm);
    }
    mvwprintw(menu_win, y++, x, "radius  [m]: %-7.2f", radius);
    y++;
    if(go) {
        mvwprintw(menu_win, y++, x, "moving: yes");
    } else {
        mvwprintw(menu_win, y++, x, "moving: no ");
    }

	wrefresh(menu_win);
}

std::pair<double, double>
convert_drive_arc_to_motor_rpms(float arc_speed, float arc_radius) {
    // Clamp speed to [-MAX_SPEED, MAX_SPEED]
    double speed = std::min<double>(MAX_SPEED, arc_speed);
    speed = std::max<double>(-MAX_SPEED, arc_speed);

    if( speed == 0.0 ) {
        return std::make_pair(0.0, 0.0);
    }

    // If the radius is zero, set it to 1e-6.
    double radius = (arc_radius == 0.0f) ? 1e-6 : arc_radius;

    double angular_speed = speed / radius;

    double vr = angular_speed * (radius + ROVER_WIDTH / 2.0);
    double vl = angular_speed * (radius - ROVER_WIDTH / 2.0);

    if( std::abs(radius) < ROVER_WIDTH / 2.0 ) {
        double old_vr = vr;
        double old_vl = vl;

        if( std::abs(vr) > std::abs(vl) ) {
            vr = speed * vr / std::abs(vr);
            vl = speed * vl / std::abs(old_vr);
        } else {
            vl = speed * vl / std::abs(vl);
            vr = speed * vr / std::abs(old_vl);
        }
    }
    const double rpm_l = vl * 60.0 / (M_PI*WHEEL_DIAM);
    const double rpm_r = vr * 60.0 / (M_PI*WHEEL_DIAM);
    return std::make_pair(rpm_l, rpm_r);
}

int main(int argc, char** argv)
{
    if( argc > 2 ) {
        printf("usage: pitdriver <logfile.csv>\n");
        return 0;
    }

    pr::WheelController motor_controller;

    std::string logfile = "/dev/null";
    if( argc == 2) { logfile = argv[1]; }
    std::ofstream logstream(logfile);

    logstream << "time_ms, "
              << "fl_rpm, fr_rpm, rl_rpm, rr_rpm, "
              << "fl_amps, fr_amps, rl_amps, rr_amps, "
              << "fl_rpm_commanded, fr_rpm_commanded, rl_rpm_commanded, rr_rpm_commanded" << std::endl;


    WINDOW *menu_win;
    double speed_rpm = 2;
    std::vector<double> radiuses = {
        -0.01, -0.1, -1.0, -2.0, -4.0, 1000.0, 4.0, 2.0, 1.0, 0.1, 0.01 };
    int rad_idx = radiuses.size()/2;
    double radius = radiuses[rad_idx];
    bool go = false;

	initscr();
	clear();
	noecho();
	cbreak();	/* Line buffering disabled. pass on everything */
    set_escdelay(1);
    curs_set(0);
	startx = (80 - WIDTH) / 2;
	starty = (24 - HEIGHT) / 2;
		
	menu_win = newwin(HEIGHT, WIDTH, starty, startx);
	keypad(menu_win, TRUE);
    nodelay(menu_win, TRUE);
	mvprintw(0, 0, "PITDRIVER");
	mvprintw(1, 0, "   Arrow keys set speed and radius");
    mvprintw(2, 0, "   Enter to go/stop");
    mvprintw(3, 0, "   ESC to quit");
    mvprintw(5, 0, "Logging to %s", logfile.c_str());
	refresh();
	print_menu(menu_win, speed_rpm, radius, go);

    long long int start_time_ms = time_in_ms();

    bool quit = false;
	while(!quit)
	{
        int c = wgetch(menu_win);
		switch(c)
		{
            case KEY_UP:
                speed_rpm = std::min<double>(speed_rpm+1, 24.0);
				break;
			case KEY_DOWN:
                speed_rpm = std::max<double>(speed_rpm-1, -24.0);
				break;
            case KEY_LEFT:
                rad_idx = std::min<int>(rad_idx+1, radiuses.size()-1);
                radius = radiuses[rad_idx];
				break;
            case KEY_RIGHT:
                rad_idx = std::max<int>(rad_idx-1, 0);
                radius = radiuses[rad_idx];
				break;
			case 10:    /* Enter Key */
                go = !go;
				break;
            case 27:
				quit = true;
				break;
			default:
				break;
		}

		print_menu(menu_win, speed_rpm, radius, go);

        // Set motor speeds
        auto [l, r] = convert_drive_arc_to_motor_rpms(RPM2MPS(speed_rpm), radius);
        double fl_rpm_commanded = l;
        double fr_rpm_commanded = r;
        double rl_rpm_commanded = l;
        double rr_rpm_commanded = r;

        mvwprintw(menu_win, 8, 0, "   %f   %f   ", fl_rpm_commanded, fr_rpm_commanded);
        wrefresh(menu_win);

        if( !go ) {
            fl_rpm_commanded = 0.0;
            fr_rpm_commanded = 0.0;
            rl_rpm_commanded = 0.0;
            rr_rpm_commanded = 0.0;
        }

        try {
            motor_controller.set_front_left_rpm(fl_rpm_commanded);
            motor_controller.set_front_right_rpm(fr_rpm_commanded);
            motor_controller.set_rear_left_rpm(rl_rpm_commanded);
            motor_controller.set_rear_right_rpm(rr_rpm_commanded);

            // Query motor info
            double fl_rpm = motor_controller.get_front_left_rpm();
            double fr_rpm = motor_controller.get_front_right_rpm();
            double rl_rpm = motor_controller.get_rear_left_rpm();
            double rr_rpm = motor_controller.get_rear_right_rpm();

            double fl_amps = motor_controller.get_front_left_amps();
            double fr_amps = motor_controller.get_front_right_amps();
            double rl_amps = motor_controller.get_rear_left_amps();
            double rr_amps = motor_controller.get_rear_right_amps();

            // Log to file
            logstream << time_in_ms()-start_time_ms << ",";
            logstream << fl_rpm << "," << fr_rpm << ",";
            logstream << rl_rpm << "," << rr_rpm << ",";
            logstream << fl_amps << "," << fr_amps << ",";
            logstream << rl_amps << "," << rr_amps << ",";
            logstream << fl_rpm_commanded << "," << fr_rpm_commanded << ",";
            logstream << rl_rpm_commanded << "," << rr_rpm_commanded << "\n";
        } catch (const std::exception& e) {

        }
	}	

	clrtoeol();
	refresh();
	endwin();
	return 0;
}


