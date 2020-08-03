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

        ImageRGB capture(long int exposure_us);

    private:
        Spinnaker::SystemPtr  pSystem = nullptr;
        Spinnaker::CameraPtr     pCam = nullptr;
        Spinnaker::CameraList           camList;
};

} // namespace pr
