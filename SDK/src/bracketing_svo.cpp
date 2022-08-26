///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2022, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

/****************************************************************************************
** This sample shows how to record video in Stereolabs SVO format.					   **
** SVO video files can be played with the ZED API and used with its different modules  **
*****************************************************************************************/

// Sample includes
#include "utils.hpp"

// Using namespace
using namespace sl;
using namespace std;

int idx_exposure = 0;
std::array<int, 6> exposures {2, 4, 8, 16, 32, 64};

int main(int argc, char **argv) {

    if (argc < 2) {
        cout << "Usage : Only the path of the output SVO file should be passed as argument.\n";
        return EXIT_FAILURE;
    }

    // Create a ZED camera
    Camera zed;

    // Set configuration parameters for the ZED
    InitParameters init_parameters;
    init_parameters.camera_resolution = RESOLUTION::HD720;
    init_parameters.camera_fps = 30;
    init_parameters.depth_mode = DEPTH_MODE::NONE;
    parseArgs(argc,argv,init_parameters);

    // Open the camera
    auto returned_state  = zed.open(init_parameters);
    if (returned_state != ERROR_CODE::SUCCESS) {
        print("Camera Open", returned_state, "Exit program.");
        return EXIT_FAILURE;
    }

    // Enable recording with the filename specified in argument
    String path_output(argv[1]);
    returned_state = zed.enableRecording(RecordingParameters(path_output, SVO_COMPRESSION_MODE::H264_LOSSLESS));
    if (returned_state != ERROR_CODE::SUCCESS) {
        print("Recording ZED : ", returned_state);
        zed.close();
        return EXIT_FAILURE;
    }

    // Start recording SVO, stop with Ctrl-C command
    print("SVO is Recording, use Ctrl-C to stop." );
    SetCtrlHandler();
    int frames_recorded = 0;
    sl::RecordingStatus rec_status;

    Resolution image_size = zed.getCameraInformation().camera_configuration.resolution;
    Mat image_zed(image_size.width, image_size.height, MAT_TYPE::U8_C4);
    cv::Mat image_ocv = slMat2cvMat(image_zed);

    while (!exit_app) {
        if(exit_app == 1){
        cout << "Exit app == 1" << endl;
        }
        zed.setCameraSettings(sl::VIDEO_SETTINGS::EXPOSURE, exposures[idx_exposure]);
        idx_exposure++;
        if (idx_exposure >= exposures.size())
        {
            idx_exposure = 0;
        }
        if (zed.grab() == ERROR_CODE::SUCCESS) {
            // Each new frame is added to the SVO file
            rec_status = zed.getRecordingStatus();
            if (rec_status.status)
                frames_recorded++;
            // print("Frame count: " +to_string(frames_recorded));

            zed.retrieveImage(image_zed, VIEW::LEFT);
            cv::imshow("Image Bracketing", image_ocv);
        }
    }

    // Stop recording
    zed.disableRecording();
    zed.close();
    return EXIT_SUCCESS;
}