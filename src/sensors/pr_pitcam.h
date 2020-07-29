#pragma once
#include "Spinnaker.h"

using namespace Spinnaker;

namespace pr {

class PitCamera {
    public:

        struct ImageRGB {};

        PitCamera();
        ~PitCamera();

        ImageRGB capture(long int exposure_us);

    private:
        Spinnaker::SystemPtr  pSystem = nullptr;
        Spinnaker::CameraPtr     pCam = nullptr;
        Spinnaker::CameraList           camList;
};

} // namespace pr
