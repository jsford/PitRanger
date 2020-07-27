#pragma once

#include <string>
#include <xsdevice/deviceclass.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

namespace pr {

struct IMU_Data {
    Eigen::Vector3d        linear_velocity;
    Eigen::Vector3d        linear_acceleration;
    Eigen::Quaterniond     orientation;
    Eigen::Vector3d        angular_velocity;
    Eigen::Vector3d        angular_acceleration;
};

class IMU {
    public:
    IMU(const std::string& portName, const int baudRate);
    ~IMU();

    [[nodiscard]] bool Connect();
    std::vector<IMU_Data> Poll();

    private:
    bool connected = false;
    DeviceClass xsens_device;
    std::string portName;
    const int baudRate = 115200;
    XsByteArray data;
    XsMessageArray msgs;
    XsPortInfo mtPort;
};


} // namespace pr
