#include "imu.h"
#include "log.h"
#include <fmt/format.h>

#include <xsdevice/deviceclass.h>
#include <xsens/xsdatapacket.h>
#include <xsens/xstime.h>
#include <xcommunication/legacydatapacket.h>
#include <xcommunication/int_xsdatapacket.h>

#include <iostream>
#include <iomanip>

namespace pr {

IMU::IMU(const std::string& portName, const int baudRate) :
    portName(portName), baudRate(baudRate) {}

[[nodiscard]] bool IMU::Connect() {
    // Use the port name and baud for this device.
    mtPort = XsPortInfo(portName, XsBaud::numericToRate(baudRate));

    // Open the port.
    pr::log_info("Opening port...\n");
    if (!xsens_device.openPort(mtPort)) {
        pr::log_error("Could not open port. Aborting.");
        return false;
    }

    // Put the device in configuration mode
    pr::log_info("Putting xsens_device into configuration mode...\n");
    if (!xsens_device.gotoConfig()) // Put the device into configuration mode before configuring the device
    {
        pr::log_error("Could not put device into configuration mode. Aborting.");
        return false;
    }

    // Request the device Id to check the device type
    mtPort.setDeviceId(xsens_device.getDeviceId());

    // Check if we have an MTmk4 device
    if (!mtPort.deviceId().isMtMk4())
    {
        pr::log_error("No MTmk4 device found. Aborting.");
        return false;
    }

    // Configure the device.
    XsOutputConfigurationArray cfg_array;

    //cfg_array.push_back( XsOutputConfiguration(XDI_TimestampGroup, 0) );
    //cfg_array.push_back( XsOutputConfiguration(XDI_OrientationGroup, 0) );
    cfg_array.push_back( XsOutputConfiguration(XDI_Quaternion, 0) );
    cfg_array.push_back( XsOutputConfiguration(XDI_VelocityXYZ, 0) );
    //cfg_array.push_back( XsOutputConfiguration(XDI_VelocityGroup, 0) );
    //cfg_array.push_back( XsOutputConfiguration(XDI_AccelerationGroup, 0) );
    //cfg_array.push_back( XsOutputConfiguration(XDI_AngularVelocityGroup, 0) );

    if (!xsens_device.setOutputConfiguration(cfg_array)) {
        pr::log_error("Could not configure XSens IMU.");
        return false;
    }

    pr::log_info(fmt::format("Found a device with id: {} @ port: {}, baudrate: {}\n",
                              mtPort.deviceId().toString().toStdString(),
                              portName, baudRate));

    // Put the device in measurement mode
    pr::log_info("Putting device into measurement mode...\n");
    if (!xsens_device.gotoMeasurement())
    {
        pr::log_error("Could not put device into measurement mode. Aborting.");
        return false;
    }
    connected = true; // We made it!
    pr::log_info("Connected to XSens IMU.\n");
    return true;
}

std::vector<IMU_Data> IMU::Poll() {
    std::vector<IMU_Data> imuDataVec;

    if (!connected) {
        pr::log_warn("Attempting to poll the IMU without connecting first.\n");
        return imuDataVec;
    }

    xsens_device.readDataToBuffer(data);
    xsens_device.processBufferedData(data, msgs);

    for (XsMessageArray::iterator it = msgs.begin(); it != msgs.end(); ++it)
    {
        // Retrieve a packet
        XsDataPacket packet;
        if ((*it).getMessageId() == XMID_MtData) {
            LegacyDataPacket lpacket(1, false);
            lpacket.setMessage((*it));
            lpacket.setXbusSystem(false);
            lpacket.setDeviceId(mtPort.deviceId(), 0);
            lpacket.setDataFormat(XOM_Orientation, XOS_OrientationMode_Quaternion,0);	//lint !e534
            XsDataPacket_assignFromLegacyDataPacket(&packet, &lpacket, 0);
        }
        else if ((*it).getMessageId() == XMID_MtData2) {
            packet.setMessage((*it));
            packet.setDeviceId(mtPort.deviceId());
        }

        XsQuaternion quaternion = packet.orientationQuaternion();

        IMU_Data imuData;
        imuData.orientation.w() = quaternion.w();
        imuData.orientation.x() = quaternion.x();
        imuData.orientation.y() = quaternion.y();
        imuData.orientation.z() = quaternion.z();

        fmt::print("Contains Velocity: {}\n", packet.containsVelocity());
        fmt::print("Contains {} Items\n", packet.itemCount());
        //auto velocity = packet.velocity();
        //imuData.linear_velocity.x() = velocity.at(0);
        //imuData.linear_velocity.y() = velocity.at(1);
        //imuData.linear_velocity.z() = velocity.at(2);

        //auto lin_acc = packet.rawAccelerationConverted();
        //imuData.linear_acceleration.x() = lin_acc.at(0);
        //imuData.linear_acceleration.y() = lin_acc.at(1);
        //imuData.linear_acceleration.z() = lin_acc.at(2);

        imuDataVec.push_back(imuData);
    }
    msgs.clear();
    return imuDataVec;
}

IMU::~IMU() {
    // Close port
    pr::log_info("Closing port...\n");
    xsens_device.close();
}

} // namespace pr
