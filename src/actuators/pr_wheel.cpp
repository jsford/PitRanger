#include "pr_wheel.h"
#include "pr_log.h"
#include "pr_time.h"
#include <Roboteq.h>
#include <experimental/filesystem>
#include <fmt/format.h>

using namespace roboteq;

namespace pr {

namespace {

int check_port_for_roboteq(const std::string &port_name) {
  RoboteqDevice dev;
  int status = dev.Connect(port_name);
  if (status != RQ_SUCCESS) {
    return -1;
  }

  int cnod = 0;
  if (dev.GetConfig(_CNOD, cnod) != RQ_SUCCESS) {
    return -1;
  }
  dev.Disconnect();
  return cnod;
}

// Search /dev/tty* for roboteq devices.
// Return: A hashmap mapping Roboteq CAN IDS to port names.
std::unordered_map<int, std::string> find_roboteq_ports() {
  namespace fs = std::experimental::filesystem;

  std::unordered_map<int, std::string> roboteq_ports;

  // Iterate over all files in /dev/
  for (const auto &p : fs::directory_iterator("/dev/")) {
    const std::string path = p.path().string();

    // Search for paths matching /dev/ttyACM*.
    if (path.find("ttyACM") != std::string::npos) {

      // Check candidate paths for roboteq devices.
      int can_id = check_port_for_roboteq(path);

      // If a roboteq is found, add it to the hashmap.
      if (can_id >= 0) {
        // If this roboteq CAN ID is not unique, raise an exception.
        if (roboteq_ports.find(can_id) != roboteq_ports.end()) {
          throw std::runtime_error("ROBOTEQ Error: Your roboteq devices have not been "
                                   "assigned unique CAN IDs.");
        }
        roboteq_ports[can_id] = path;
      }
    }
  }
  return roboteq_ports;
}

} // anonymous namespace

WheelController::WheelController() {

  // Scan for Roboteqs and connect to them. Retry 10 times at 1 second intervals.
  {
    int num_retries = 10;
    bool found_left = false;
    bool found_right = false;
    std::unordered_map<int, std::string> port_map;

    for (int retry = 0; retry < num_retries; ++retry) {
      // Scan for roboteqs.
      port_map = find_roboteq_ports();

      found_left = port_map.find(WHEELS_LEFT_ROBOTEQ_CAN_ID) != port_map.end();
      found_right = port_map.find(WHEELS_RIGHT_ROBOTEQ_CAN_ID) != port_map.end();

      if (found_left && found_right) {
        break;
      }

      if (found_left && !found_right) {
        auto msg = fmt::format("Failed to find right Roboteq. Retrying... [{}/{}]\n", retry + 1,
                               num_retries);
        pr::log_warn(msg);
      } else if (!found_right && found_right) {
        auto msg = fmt::format("Failed to find left Roboteq. Retrying... [{}/{}]\n", retry + 1, num_retries);
        pr::log_warn(msg);
      } else if (!found_right && !found_right) {
        auto msg = fmt::format("Failed to find left and right Roboteqs. Retrying... [{}/{}]\n",
                               retry + 1, num_retries);
        pr::log_warn(msg);
      }
      pr::time::sleep(1);
    }

    if (!found_left) {
      auto msg =
          fmt::format("Failed to find left roboteq with CAN ID {}\n", WHEELS_LEFT_ROBOTEQ_CAN_ID);
      throw std::runtime_error(msg);
    }

    if (!found_right) {
      auto msg =
          fmt::format("Failed to find right roboteq with CAN ID {}\n", WHEELS_RIGHT_ROBOTEQ_CAN_ID);
      throw std::runtime_error(msg);
    }

    int status = left.Connect(port_map.at(WHEELS_LEFT_ROBOTEQ_CAN_ID));
    if (status != RQ_SUCCESS) {
      auto msg = fmt::format("Failed to connect to left Roboteq on port {}\n",
                             port_map.at(WHEELS_LEFT_ROBOTEQ_CAN_ID));
      throw std::runtime_error(msg);
    }

    status = right.Connect(port_map.at(WHEELS_RIGHT_ROBOTEQ_CAN_ID));
    if (status != RQ_SUCCESS) {
      auto msg = fmt::format("Failed to connect to left Roboteq on port {}\n",
                             port_map.at(WHEELS_RIGHT_ROBOTEQ_CAN_ID));
      throw std::runtime_error(msg);
    }
  }

  // Configure the Left Roboteq.
  {
    // Configure Roboteq
    disable_watchdog(left);
    use_velocity_mode(left);
    use_quadrature_encoder(left, WHEELS_ENCODER_PPR);
    set_max_vel(left, WHEELS_MAX_MOTOR_RPM);
    set_kp(left, 16.0);
    set_ki(left, 1.0);
    set_kd(left, 0.0);
  }

  // Configure the Right Roboteq.
  {
    // Configure Roboteq
    disable_watchdog(right);
    use_velocity_mode(right);
    use_quadrature_encoder(right, WHEELS_ENCODER_PPR);
    set_max_vel(right, WHEELS_MAX_MOTOR_RPM);
    set_kp(right, 16.0);
    set_ki(right, 1.0);
    set_kd(right, 0.0);
  }
}

double WheelController::set_front_right_rpm(double rpm) {
  fr_rpm = rpm;

  int fr_cmd = std::clamp<int>(fr_rpm / WHEELS_MAX_WHEEL_RPM * 1000, -1000, 1000);
  int rr_cmd = std::clamp<int>(rr_rpm / WHEELS_MAX_WHEEL_RPM * 1000, -1000, 1000);

  int status = right.SetCommand(_MOTCMD, -1 * fr_cmd, -1 * rr_cmd);
  if (status != RQ_SUCCESS) {
    auto msg = fmt::format("Failed to set front-right wheel RPM to {}.\n", rpm);
    throw std::runtime_error(msg);
  }

  return std::clamp<double>(rpm, -WHEELS_MAX_WHEEL_RPM, WHEELS_MAX_WHEEL_RPM);
}

double WheelController::set_front_left_rpm(double rpm) {
  fl_rpm = rpm;

  int fl_cmd = std::clamp<int>(fl_rpm / WHEELS_MAX_WHEEL_RPM * 1000, -1000, 1000);
  int rl_cmd = std::clamp<int>(rl_rpm / WHEELS_MAX_WHEEL_RPM * 1000, -1000, 1000);

  int status = left.SetCommand(_MOTCMD, fl_cmd, rl_cmd);
  if (status != RQ_SUCCESS) {
    auto msg = fmt::format("Failed to set front-left wheel RPM to {}.\n", rpm);
    throw std::runtime_error(msg);
  }

  return std::clamp<double>(rpm, -WHEELS_MAX_WHEEL_RPM, WHEELS_MAX_WHEEL_RPM);
}

double WheelController::set_rear_right_rpm(double rpm) {
  rr_rpm = rpm;

  int fr_cmd = std::clamp<int>(fr_rpm / WHEELS_MAX_WHEEL_RPM * 1000, -1000, 1000);
  int rr_cmd = std::clamp<int>(rr_rpm / WHEELS_MAX_WHEEL_RPM * 1000, -1000, 1000);

  int status = right.SetCommand(_MOTCMD, -1 * fr_cmd, -1 * rr_cmd);
  if (status != RQ_SUCCESS) {
    auto msg = fmt::format("Failed to set rear-right wheel RPM to {}.\n", rpm);
    throw std::runtime_error(msg);
  }

  return std::clamp<double>(rpm, -WHEELS_MAX_WHEEL_RPM, WHEELS_MAX_WHEEL_RPM);
}

double WheelController::set_rear_left_rpm(double rpm) {
  rl_rpm = rpm;

  int fl_cmd = std::clamp<int>(fl_rpm / WHEELS_MAX_WHEEL_RPM * 1000, -1000, 1000);
  int rl_cmd = std::clamp<int>(rl_rpm / WHEELS_MAX_WHEEL_RPM * 1000, -1000, 1000);

  int status = left.SetCommand(_MOTCMD, fl_cmd, rl_cmd);
  if (status != RQ_SUCCESS) {
    auto msg = fmt::format("Failed to set rear-left wheel RPM to {}.\n", rpm);
    throw std::runtime_error(msg);
  }

  return std::clamp<double>(rpm, -WHEELS_MAX_WHEEL_RPM, WHEELS_MAX_WHEEL_RPM);
}

double WheelController::get_front_right_rpm() {
  int enc = get_front_right_encoder();
  return -1 * enc / (double)WHEELS_MOTOR_GEAR_RATIO;
}
double WheelController::get_front_left_rpm() {
  int enc = get_front_left_encoder();
  return enc / (double)WHEELS_MOTOR_GEAR_RATIO;
}
double WheelController::get_rear_right_rpm() {
  int enc = get_rear_right_encoder();
  return -1 * enc / (double)WHEELS_MOTOR_GEAR_RATIO;
}
double WheelController::get_rear_left_rpm() {
  int enc = get_rear_left_encoder();
  return enc / (double)WHEELS_MOTOR_GEAR_RATIO;
}

int WheelController::get_front_right_encoder() {
  int pos = 0;
  int status = right.GetValue(_F, WHEELS_RIGHT_FRONT_CHANNEL, pos);
  if (status != RQ_SUCCESS) {
    auto msg = fmt::format("Failed to get front right wheel encoder value.");
    throw std::runtime_error(msg);
  }
  return pos;
}
int WheelController::get_front_left_encoder() {
  int pos;
  int status = left.GetValue(_F, WHEELS_LEFT_FRONT_CHANNEL, pos);
  if (status != RQ_SUCCESS) {
    auto msg = fmt::format("Failed to get front left wheel encoder value.");
    throw std::runtime_error(msg);
  }
  return pos;
}
int WheelController::get_rear_right_encoder() {
  int pos;
  int status = right.GetValue(_F, WHEELS_RIGHT_REAR_CHANNEL, pos);
  if (status != RQ_SUCCESS) {
    auto msg = fmt::format("Failed to get rear right wheel encoder value.");
    throw std::runtime_error(msg);
  }
  return pos;
}
int WheelController::get_rear_left_encoder() {
  int pos;
  int status = left.GetValue(_F, WHEELS_LEFT_REAR_CHANNEL, pos);
  if (status != RQ_SUCCESS) {
    auto msg = fmt::format("Failed to get rear left wheel encoder value.");
    throw std::runtime_error(msg);
  }
  return pos;
}

void WheelController::disable_watchdog(RoboteqDevice &dev) {
  int status = dev.SetConfig(_RWD, 0);
  if (status != RQ_SUCCESS) {
    auto msg = fmt::format("Failed to disable roboteq watchdog.\n");
    throw std::runtime_error(msg);
  }
}

void WheelController::disable_loop_error_detection(RoboteqDevice &dev) {
  int status = dev.SetConfig(_CLERD, 1, 0);
  if (status != RQ_SUCCESS) {
    auto msg = fmt::format("Failed to disable loop error detection.\n");
    throw std::runtime_error(msg);
  }
  status = dev.SetConfig(_CLERD, 2, 0);
  if (status != RQ_SUCCESS) {
    auto msg = fmt::format("Failed to disable loop error detection.\n");
    throw std::runtime_error(msg);
  }
}

void WheelController::use_velocity_mode(RoboteqDevice &dev) {
  int status = dev.SetConfig(_MMOD, 1, 1);
  if (status != RQ_SUCCESS) {
    auto msg = fmt::format("Failed to set operating mode to closed-loop speed.\n");
    throw std::runtime_error(msg);
  }
  status = dev.SetConfig(_MMOD, 2, 1);
  if (status != RQ_SUCCESS) {
    auto msg = fmt::format("Failed to set operating mode to closed-loop speed.\n");
    throw std::runtime_error(msg);
  }
}
void WheelController::use_quadrature_encoder(RoboteqDevice &dev, int encoder_ppr) {
  // Set Encoder Mode to Feedback
  {
    int status = dev.SetConfig(_EMOD, 1, 1 * 16 + 2);
    if (status != RQ_SUCCESS) {
      auto msg = fmt::format("Failed to set encoder mode to 'Feedback'.\n");
      throw std::runtime_error(msg);
    }
    status = dev.SetConfig(_EMOD, 2, 2 * 16 + 2);
    if (status != RQ_SUCCESS) {
      auto msg = fmt::format("Failed to set encoder mode to 'Feedback'.\n");
      throw std::runtime_error(msg);
    }
  }

  // Set Encoder Pulses Per Revolution
  {
    int status = dev.SetConfig(_EPPR, 1, encoder_ppr);
    if (status != RQ_SUCCESS) {
      auto msg = fmt::format("Failed to set encoder pulse per revolution.\n");
      throw std::runtime_error(msg);
    }
    status = dev.SetConfig(_EPPR, 2, encoder_ppr);
    if (status != RQ_SUCCESS) {
      auto msg = fmt::format("Failed to set encoder pulse per revolution.\n");
      throw std::runtime_error(msg);
    }
  }
}
void WheelController::set_max_vel(RoboteqDevice &dev, int motor_rpm) {
  // Set max motor RPM.
  {
    int status = dev.SetConfig(_MXRPM, 1, motor_rpm);
    if (status != RQ_SUCCESS) {
      auto msg = fmt::format("Failed to set max motor velocity.\n");
      throw std::runtime_error(msg);
    }
    status = dev.SetConfig(_MXRPM, 2, motor_rpm);
    if (status != RQ_SUCCESS) {
      auto msg = fmt::format("Failed to set max motor velocity.\n");
      throw std::runtime_error(msg);
    }
  }

  // Set max motor acceleration.
  {
    int status = dev.SetConfig(_MAC, 1, motor_rpm * 10);
    if (status != RQ_SUCCESS) {
      auto msg = fmt::format("Failed to set max motor acceleration.\n");
      throw std::runtime_error(msg);
    }
    status = dev.SetConfig(_MAC, 2, motor_rpm * 10);
    if (status != RQ_SUCCESS) {
      auto msg = fmt::format("Failed to set max motor acceleration.\n");
      throw std::runtime_error(msg);
    }
  }

  // Set max motor deceleration.
  {
    int status = dev.SetConfig(_MDEC, 1, motor_rpm * 10);
    if (status != RQ_SUCCESS) {
      auto msg = fmt::format("Failed to set max motor deceleration.\n");
      throw std::runtime_error(msg);
    }
    status = dev.SetConfig(_MDEC, 2, motor_rpm * 10);
    if (status != RQ_SUCCESS) {
      auto msg = fmt::format("Failed to set max motor deceleration.\n");
      throw std::runtime_error(msg);
    }
  }
}
void WheelController::set_kp(RoboteqDevice &dev, int kp) {
  int status = dev.SetConfig(_KP, 1, kp * 10);
  if (status != RQ_SUCCESS) {
    auto msg = fmt::format("Failed to set motor Kp.\n");
    throw std::runtime_error(msg);
  }

  status = dev.SetConfig(_KP, 2, kp * 10);
  if (status != RQ_SUCCESS) {
    auto msg = fmt::format("Failed to set motor Kp.\n");
    throw std::runtime_error(msg);
  }
}
void WheelController::set_ki(RoboteqDevice &dev, int ki) {
  int status = dev.SetConfig(_KI, 1, ki * 10);
  if (status != RQ_SUCCESS) {
    auto msg = fmt::format("Failed to set motor Ki.\n");
    throw std::runtime_error(msg);
  }

  status = dev.SetConfig(_KI, 2, ki * 10);
  if (status != RQ_SUCCESS) {
    auto msg = fmt::format("Failed to set motor Ki.\n");
    throw std::runtime_error(msg);
  }
}
void WheelController::set_kd(RoboteqDevice &dev, int kd) {
  int status = dev.SetConfig(_KD, 1, kd * 10);
  if (status != RQ_SUCCESS) {
    auto msg = fmt::format("Failed to set motor Kd.\n");
    throw std::runtime_error(msg);
  }

  status = dev.SetConfig(_KD, 2, kd * 10);
  if (status != RQ_SUCCESS) {
    auto msg = fmt::format("Failed to set motor Kd.\n");
    throw std::runtime_error(msg);
  }
}

} // namespace pr
