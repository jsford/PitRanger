#include "pr_pitcam.h"
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;

namespace pr {


PitCamera::PitCamera() {

    pSystem = System::GetInstance();
    camList = pSystem->GetCameras();

    if( camList.GetSize() < 1 ) {
        camList.Clear();
        pSystem->ReleaseInstance();
        throw std::runtime_error("Failed to find a Spinnaker camera.");
    }

    pCam = camList.GetByIndex(0);
}

PitCamera::~PitCamera() {
    pCam = nullptr;
    camList.Clear();
    pSystem->ReleaseInstance();
}

PitCamera::ImageRGB
PitCamera::capture(long int exposure_us) {
    // Initialize the camera.
    pCam->Init();

    // Retrieve the TL device nodemap.
    INodeMap& nodeMapTLDevice = pCam->GetTLDeviceNodeMap();

    // Retrieve the GenICam nodemap.
    INodeMap& nodeMap = pCam->GetNodeMap();

        // Retrieve enumeration node from nodemap
        CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
        if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
        {
            throw std::runtime_error("Unable to set acquisition mode to 'single'. Aborting...");
        }

        // Retrieve entry node from enumeration node
        CEnumEntryPtr ptrAcquisitionModeSingle = ptrAcquisitionMode->GetEntryByName("SingleFrame");
        if (!IsAvailable(ptrAcquisitionModeSingle) || !IsReadable(ptrAcquisitionModeSingle))
        {
            throw std::runtime_error("Unable to set acquisition mode to 'single'. Aborting...");
        }

        // Retrieve integer value from entry node
        const int64_t acquisitionModeSingle = ptrAcquisitionModeSingle->GetValue();

        // Set integer value from entry node as new value of enumeration node
        ptrAcquisitionMode->SetIntValue(acquisitionModeSingle);

        pCam->BeginAcquisition();
        {
            // Retrieve next received image
            ImagePtr pResultImage = pCam->GetNextImage(1000);

            // Ensure image completion
            if (pResultImage->IsIncomplete())
            {
                // Retrieve and print the image status description
                std::cout << "Image incomplete: " << Image::GetImageStatusDescription(pResultImage->GetImageStatus())
                     << "..." << std::endl
                     << std::endl;
            }
            else
            {
                const size_t width = pResultImage->GetWidth();
                const size_t height = pResultImage->GetHeight();
                std::cout << "Grabbed image width = " << width << ", height = " << height << std::endl;
            }
            pResultImage->Release();
        }
        pCam->EndAcquisition();

    // Deinitialize the camera.
    pCam->DeInit();

    PitCamera::ImageRGB image;

    return image;
}

} // namespace pr

