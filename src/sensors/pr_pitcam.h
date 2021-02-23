#pragma once
#include "Spinnaker.h"
#include <vector>

using namespace Spinnaker;

namespace pr {

class PitCamera {
    public:

        struct ImageRGB {
            int rows, cols;
            std::vector<uint8_t> data;
        };

        PitCamera();
        ~PitCamera();

        // Calculate optimal exposure setting [us].
        // NOTE: This is VERY SLOW.
        int get_autoexposure();

        // Get the max. exposure time available on this camera [us].
        int get_max_exposure();

        // Get the min. exposure time available on this camera [us].
        int get_min_exposure();

        // Get the current exposure setting on this camera [us].
        int get_exposure();

        // Take a picture with the given exposure [us].
        ImageRGB capture(int exposure_us);

    private:
        Spinnaker::SystemPtr  pSystem = nullptr;
        Spinnaker::CameraPtr     pCam = nullptr;
        Spinnaker::CameraList           camList;

        // Set the camera exposure [us].
        void set_exposure(int exposure_us);

        // Put the camera into auto exposure mode.
        void use_auto_exposure();

        // Put the camera into manual exposure mode.
        void use_manual_exposure();

        // Limit the link speed to prevent packet loss.
        void set_device_link_throughput(int MBps);
};

} // namespace pr
