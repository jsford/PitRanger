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

    // Get Device Link Throughput Limit Setting
    CIntegerPtr ptrDeviceLinkThroughputLimit = pCam->GetNodeMap().GetNode("DeviceLinkThroughputLimit");
    if (!IsAvailable(ptrDeviceLinkThroughputLimit) || !IsWritable(ptrDeviceLinkThroughputLimit)) {
        throw std::runtime_error("Unable to set device link throughput limit (node retrieval). Aborting...");
    }
    // Limit Device Link Throughput to 50M
    ptrDeviceLinkThroughputLimit->SetValue(DEVICE_LINK_THROUGHPUT_LIMIT_MBPS);
}

PitCamera::~PitCamera() {
    // Deinitialize the camera.
    pCam->DeInit();

    pCam = nullptr;
    camList.Clear();
    pSystem->ReleaseInstance();
}

PitCamera::ImageRGB
PitCamera::capture(long int exposure_us) {
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
            pr::log_warn("PitCamera image is incomplete. Retrying... [{}/{}]\n", retry+1, num_retries);
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

} // namespace pr

