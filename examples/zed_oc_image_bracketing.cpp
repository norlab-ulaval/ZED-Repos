///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2021, STEREOLABS.
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

// ----> Includes
#include "videocapture.hpp"
#include "ocv_display.hpp"

#include <iostream>
#include <iomanip>

#include <opencv2/opencv.hpp>
// <---- Includes

// #define TEST_FPS 1


// ************* JM ***************
int stride = 1;     // Number of frames to skip between exposure changes
std::array<int, 5> exposures {0, 10, 20, 30, 40};
std::string resolution = "HD1080";
int framerate = 30;
// ********************************

// The main function
int main(int argc, char *argv[])
{
    // ----> Silence unused warning
    (void)argc;
    (void)argv;
    // <---- Silence unused warning

    // ----> Create Video Capture
    sl_oc::video::VideoParams params;

    // *********** OG *********************************************
    int width = 0;
    int height = 0; 
    // SET RESOLUTION
    if(resolution == "VGA") // 15, 30, 60 or 100 FPS
    {
        params.res = sl_oc::video::RESOLUTION::VGA;
        width = 672;
        height = 376;
    } else if(resolution == "HD720") // 15, 30 or 60 FPS
    {
        params.res = sl_oc::video::RESOLUTION::HD720;
        width = 1280;
        height = 720;
    } else if (resolution == "HD1080") // 15 or 30 FPS
    {
        params.res = sl_oc::video::RESOLUTION::HD1080;
        width = 1920;
        height = 1080;
    } else if (resolution == "2K") // 15FPS
    {
        params.res = sl_oc::video::RESOLUTION::HD2K;
        width = 2208;
        height = 1242;
    }
    // SET FRAME RATE
    if(framerate == 15)
    {
        params.fps = sl_oc::video::FPS::FPS_15;
    } else if(framerate == 30)
    {
        params.fps = sl_oc::video::FPS::FPS_30;
    } else if(framerate == 60)
    {
        params.fps = sl_oc::video::FPS::FPS_60;
    } else if(framerate == 100)
    {
        params.fps = sl_oc::video::FPS::FPS_100;
    }
    //cv::VideoWriter video_writer("out_bracketing.avi", cv::VideoWriter::fourcc('M','J','P','G'), framerate, cv::Size(2*width,height));
    // **************************************************************************
    
    sl_oc::video::VideoCapture cap(params);
    if( !cap.initializeVideo() )
    {
        std::cerr << "Cannot open camera video capture" << std::endl;
        std::cerr << "See verbosity level for more details." << std::endl;

        return EXIT_FAILURE;
    }
    std::cout << "Connected to camera sn: " << cap.getSerialNumber() << std::endl;
    // <---- Create Video Capture



#ifdef TEST_FPS
    // Timestamp to check FPS
    double lastTime = static_cast<double>(getSteadyTimestamp())/1e9;
    // Frame timestamp to check FPS
    uint64_t lastFrameTs = 0;
#endif

    // Infinite video grabbing loop
    //for (int i = 0; i < images.size(); i)
    int i = 0;
    while(1)
    {
        // Get last available frame
        const sl_oc::video::Frame frame = cap.getLastFrame();

        // ----> If the frame is valid we can display it
        if(frame.data!=nullptr)
        {
            if (i%stride == 0) {
            // ************* JM ***************
                int exposure = exposures[(i/stride) % exposures.size()];
                cap.setExposure(sl_oc::video::CAM_SENS_POS::RIGHT, exposure);
                // cap.setExposure(sl_oc::video::CAM_SENS_POS::LEFT, exposure);
            }
            // ********************************
#ifdef TEST_FPS
            if(lastFrameTs!=0)
            {
                // ----> System time
                double now = static_cast<double>(getSteadyTimestamp())/1e9;
                double elapsed_sec = now - lastTime;
                lastTime = now;
                std::cout << "[System] Frame period: " << elapsed_sec << "sec - Freq: " << 1./elapsed_sec << " Hz" << std::endl;
                // <---- System time

                // ----> Frame time
                double frame_dT = static_cast<double>(frame.timestamp-lastFrameTs)/1e9;
                std::cout << "[Camera] Frame period: " << frame_dT << "sec - Freq: " << 1./frame_dT << " Hz" << std::endl;
                // <---- Frame time
            }
            lastFrameTs = frame.timestamp;
#endif

            // ----> Conversion from YUV 4:2:2 to BGR for visualization
            cv::Mat frameYUV = cv::Mat( frame.height, frame.width, CV_8UC2, frame.data );
            cv::Mat frameBGR;
            cv::cvtColor(frameYUV,frameBGR,cv::COLOR_YUV2BGR_YUYV);
            // <---- Conversion from YUV 4:2:2 to BGR for visualization

            // Show frame
            sl_oc::tools::showImage( "Stream RGB", frameBGR, params.res  );
	
	    //std::cout << "/image_left/left_"+ std::to_string(i) +".png" << std::endl;
	    cv::imwrite("/media/snowxavier/norlab_olivier/zed_xavier_agx/data/images/image_"+ std::to_string(i) +".bmp", frameBGR);
            //images[i] = frameBGR;      // **************** JM *****************
            i++;
        }
        // <---- If the frame is valid we can display it

        // ----> Keyboard handling
        int key = cv::waitKey( 5 );
        if(key=='q' || key=='Q') // Quit
            break;
        // <---- Keyboard handling
    }

    //for (cv::Mat image: images)
    //{
    //    video_writer.write(image);
    //} 

    return EXIT_SUCCESS;
}


