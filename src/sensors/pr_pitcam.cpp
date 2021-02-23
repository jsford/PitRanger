#include "pr_pitcam.h"
#include "pr_log.h"
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <fmt/format.h>

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;

namespace pr {

const int64_t DEVICE_LINK_THROUGHPUT_LIMIT_MBPS = 30000000;

PitCamera::PitCamera() {

    pSystem = System::GetInstance();
    camList = pSystem->GetCameras();

    if( camList.GetSize() < 1 ) {
        camList.Clear();
        pSystem->ReleaseInstance();
        throw std::runtime_error("Failed to find a Spinnaker camera.");
    }

    pCam = camList.GetByIndex(0);

    // Initialize the camera.
    pCam->Init();

    // Set device link throughput.
    set_device_link_throughput(DEVICE_LINK_THROUGHPUT_LIMIT_MBPS);

    // Calculate appropriate exposure and use it.
    int exp_us = get_autoexposure();
    set_exposure(exp_us);
}

PitCamera::~PitCamera() {
    // Deinitialize the camera.
    pCam->DeInit();

    pCam = nullptr;
    camList.Clear();
    pSystem->ReleaseInstance();
}

void PitCamera::set_device_link_throughput(int MBps) {
    CIntegerPtr ptrDeviceLinkThroughputLimit = pCam->GetNodeMap().GetNode("DeviceLinkThroughputLimit");
    if (!IsAvailable(ptrDeviceLinkThroughputLimit) || !IsWritable(ptrDeviceLinkThroughputLimit)) {
        throw std::runtime_error("Unable to set device link throughput limit (node retrieval). Aborting...");
    }
    ptrDeviceLinkThroughputLimit->SetValue(MBps);
}

int PitCamera::get_autoexposure() {
    // Save current exposure so we can restore it later.
    int original_exposure = get_exposure();

    // Set the exposure to something reasonable to start with.
    set_exposure(15000);

    // Enable auto exposure and capture images to allow it to converge.
    use_auto_exposure();
    for(int i=0; i<5; ++i) {
        capture(0);
    }
    // Save the calculated auto exposure.
    int auto_exposure = get_exposure();

    // Reset to original exposure.
    use_manual_exposure();
    set_exposure(original_exposure);

    // Return the optimal exposure.
    return auto_exposure;
}

int PitCamera::get_max_exposure() {
    CFloatPtr ptrExposureTime = pCam->GetNodeMap().GetNode("ExposureTime");
    if (!IsAvailable(ptrExposureTime) || !IsReadable(ptrExposureTime)) {
        throw std::runtime_error("Unable to retrieve ExposureTime node.");
    }
    return ptrExposureTime->GetMax();
}

int PitCamera::get_min_exposure() {
    CFloatPtr ptrExposureTime = pCam->GetNodeMap().GetNode("ExposureTime");
    if (!IsAvailable(ptrExposureTime) || !IsReadable(ptrExposureTime)) {
        throw std::runtime_error("Unable to retrieve ExposureTime node.");
    }
    return ptrExposureTime->GetMin();
}

int PitCamera::get_exposure() {
    CFloatPtr ptrExposureTime = pCam->GetNodeMap().GetNode("ExposureTime");
    if (!IsAvailable(ptrExposureTime) || !IsReadable(ptrExposureTime)) {
        throw std::runtime_error("Unable to retrieve ExposureTime node.");
    }
    return ptrExposureTime->GetValue();
}

void PitCamera::set_exposure(int exposure_us) {
    use_manual_exposure();

    int exp_min = get_min_exposure();
    int exp_max = get_max_exposure();
    exposure_us = std::clamp(exposure_us, exp_min, exp_max);

    fmt::print("Setting exposure to {} us.\n", exposure_us);

    CFloatPtr ptrExposureTime = pCam->GetNodeMap().GetNode("ExposureTime");
    if (!IsAvailable(ptrExposureTime) || !IsReadable(ptrExposureTime)) {
        throw std::runtime_error("Unable to retrieve ExposureTime node.");
    }
    ptrExposureTime->SetValue(exposure_us);
}

PitCamera::ImageRGB
PitCamera::capture(int exposure_us) {
    if (exposure_us > 0) {
        set_exposure(exposure_us);
    }

    // Retrieve the GenICam nodemap.
    auto &nodeMap = pCam->GetNodeMap();

    // Retrieve enumeration node from nodemap
    CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
    if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
    {
        throw std::runtime_error("Unable to set acquisition mode to 'continuous'. Aborting...");
    }

    // Retrieve entry node from enumeration node
    CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
    if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous))
    {
        throw std::runtime_error("Unable to set acquisition mode to 'continuous'. Aborting...");
    }

    // Retrieve integer value from entry node
    const int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();

    // Set integer value from entry node as new value of enumeration node
    ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);

    int retry;
    const int num_retries = 10;
    for(retry=0; retry < num_retries; ++retry) {
        pCam->BeginAcquisition();
        // Retrieve next received image
        ImagePtr pResultImage = pCam->GetNextImage();

        // Ensure image completion
        if (pResultImage->IsIncomplete()) {
            pResultImage->Release();
            pCam->EndAcquisition();
        } else {
            // Convert to RGB8 (if the image is not RGB8 already)
            ImagePtr pImageRGB = pResultImage->Convert(PixelFormat_RGB8);

            // Convert Spinnaker::Image to ImageRGB
            const size_t width = pImageRGB->GetWidth();
            const size_t height = pImageRGB->GetHeight();
            const size_t channels = pImageRGB->GetNumChannels();
            const uint8_t* img_data = static_cast<const uint8_t*>(pImageRGB->GetData());

            if( pImageRGB->GetXPadding() != 0 || pImageRGB->GetYPadding() != 0 ) {
                std::runtime_error("PitCam image has unexpected nonzero padding.");
            }

            // Copy the image data into our own image format.
            PitCamera::ImageRGB image;
            image.cols = width;
            image.rows = height;
            image.data.resize(width*height*channels);

            std::memcpy(&image.data[0], img_data, channels*width*height);

            pResultImage->Release();
            pCam->EndAcquisition();

            return image;
        }
    }

    // If we used all our retries, throw an exception.
    auto msg = fmt::format("PitCam failed to capture a complete image after {} attempts.", num_retries);
    throw std::runtime_error(msg);
}

void PitCamera::use_auto_exposure() {
    CEnumerationPtr ptrExposureAuto = pCam->GetNodeMap().GetNode("ExposureAuto");
    if (!IsAvailable(ptrExposureAuto) || !IsWritable(ptrExposureAuto)) {
        throw std::runtime_error("Unable to retrieve ExposureAuto node.");
    }

    CEnumEntryPtr ptrExposureAutoContinuous = ptrExposureAuto->GetEntryByName("Continuous");
    if (!IsAvailable(ptrExposureAutoContinuous) || !IsReadable(ptrExposureAutoContinuous)) {
        throw std::runtime_error("Unable to retrieve ExposureAuto 'Continuous' entry node.");
    }
    ptrExposureAuto->SetIntValue(ptrExposureAutoContinuous->GetValue());
}

void PitCamera::use_manual_exposure() {
    CEnumerationPtr ptrExposureAuto = pCam->GetNodeMap().GetNode("ExposureAuto");
    if (!IsAvailable(ptrExposureAuto) || !IsWritable(ptrExposureAuto)) {
        throw std::runtime_error("Unable to retrieve ExposureAuto node.");
    }

    CEnumEntryPtr ptrExposureAutoOff = ptrExposureAuto->GetEntryByName("Off");
    if (!IsAvailable(ptrExposureAutoOff) || !IsReadable(ptrExposureAutoOff)) {
        throw std::runtime_error("Unable to retrieve ExposureAuto 'Off' entry node.");
    }
    ptrExposureAuto->SetIntValue(ptrExposureAutoOff->GetValue());
}

} // namespace pr

