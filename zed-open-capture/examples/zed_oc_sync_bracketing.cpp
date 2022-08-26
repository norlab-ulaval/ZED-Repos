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
#include "sensorcapture.hpp"

#include <iostream>
#include <sstream>
#include <iomanip>
#include <thread>
#include <mutex>
#include <chrono>
#include <ctime>
#include <sys/stat.h>
#include <errno.h>
#include <boost/algorithm/string.hpp>

#include <opencv2/opencv.hpp>
// <---- Includes

// ************* JM ***************
int stride = 1;     // Number of frames to skip between exposure changes
std::array<int, 4> exposures {10, 20, 40, 80};
std::string resolution = "HD720";
int framerate = 15;
std::string save_path = "/home/alienware/Documents/ZED_data/";
// ********************************


// ----> Functions
// Sensor acquisition runs at 400Hz, so it must be executed in a different thread
// void getSensorThreadFunc(sl_oc::sensors::SensorCapture* sensCap);
void getVideoThreadFunc(sl_oc::video::VideoCapture* videoCap, sl_oc::video::RESOLUTION resolution);
void saveVideoThreadFunc(std::string time);
// <---- Functions

// ----> Global variables
std::mutex imuMutex;
std::mutex imageMutex;
// std::string imuTsStr;
// std::string imuAccelStr;
// std::string imuGyroStr;
std::queue<cv::Mat> imageQueue;

// bool sensThreadStop=false;
bool videoThreadStop=false;
bool saveThreadStop=false;
uint64_t mcu_sync_ts=0;

// <---- Global variables

// The main function
int main(int argc, char *argv[])
{
    std::time_t time_point = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    char time [80];
    std::strftime(time, 80, "%Y-%m-%d_%H-%M-%S", std::localtime(&time_point));
    std::cout << time << std::endl;
    mkdir((save_path + time).c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    mkdir((save_path + time + "/images").c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    mkdir((save_path + time + "/imu").c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    // ----> Silence unused warning
    (void)argc;
    (void)argv;
    // <---- Silence unused warning

    //sl_oc::sensors::SensorCapture::resetSensorModule();
    //sl_oc::sensors::SensorCapture::resetVideoModule();

    // Set the verbose level
    sl_oc::VERBOSITY verbose = sl_oc::VERBOSITY::ERROR;

    // ----> Set the video parameters
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
    params.verbose = verbose;
    // <---- Video parameters

    // ----> Create a Video Capture object
    sl_oc::video::VideoCapture videoCap(params);
    if( !videoCap.initializeVideo(-1) )
    {
        std::cerr << "Cannot open camera video capture" << std::endl;
        std::cerr << "Try to enable verbose to get more info" << std::endl;

        return EXIT_FAILURE;
    }

    // Serial number of the connected camera
    int camSn = videoCap.getSerialNumber();

    std::cout << "Video Capture connected to camera sn: " << camSn << std::endl;
    // <---- Create a Video Capture object

    // ----> Create a Sensors Capture object
    // sl_oc::sensors::SensorCapture sensCap(verbose);
    // if( !sensCap.initializeSensors(camSn) ) // Note: we use the serial number acquired by the VideoCapture object
    // {
    //     std::cerr << "Cannot open sensors capture" << std::endl;
    //     std::cerr << "Try to enable verbose to get more info" << std::endl;

    //     return EXIT_FAILURE;
    // }
    // std::cout << "Sensors Capture connected to camera sn: " << sensCap.getSerialNumber() << std::endl;

    // Start the sensor capture thread. Note: since sensor data can be retrieved at 400Hz and video data frequency is
    // minor (max 100Hz), we use a separated thread for sensors.
    // std::thread sensThread(getSensorThreadFunc,&sensCap);
    // <---- Create Sensors Capture

    // ----> Enable video/sensors synchronization
    // videoCap.enableSensorSync(&sensCap);
    // <---- Enable video/sensors synchronization


    std::vector<std::thread> threads;
    threads.push_back(std::thread(getVideoThreadFunc, &videoCap, params.res));
    // threads.push_back(std::thread(getSensorThreadFunc, &sensCap));
    threads.push_back(std::thread(saveVideoThreadFunc, time));

    //cv::namedWindow("Stream RGB", cv::WINDOW_AUTOSIZE);

    for(auto& thread : threads){
        thread.join();
    }
    return 0;

    // Infinite grabbing loop
    //while (1)
    //{

        // <---- Display frame with info

        // ----> Keyboard handling
        //int key = cv::waitKey(1);

        //if(key != -1)
        //{
            // Quit
          //  if(key=='q' || key=='Q'|| key==27)
           // {
            //    sensThreadStop=true;
             //   videoThreadStop=true;
	//	saveThreadStop=true;
          //      for(auto& thread : threads){
            //        thread.join();
              //  }
             //   break;
         ///   }
       // }
        // <---- Keyboard handling
    //}

    //return EXIT_SUCCESS;
}

// // Sensor acquisition runs at 400Hz, so it must be executed in a different thread
// void getSensorThreadFunc(sl_oc::sensors::SensorCapture* sensCap)
// {
//     // Flag to stop the thread
//     sensThreadStop = false;

//     // Previous IMU timestamp to calculate frequency
//     uint64_t last_imu_ts = 0;

//     // Infinite data grabbing loop
//     while(!sensThreadStop)
//     {
//         // ----> Get IMU data
//         const sl_oc::sensors::data::Imu imuData = sensCap->getLastIMUData(2000);

//         // Process data only if valid
//         if(imuData.valid == sl_oc::sensors::data::Imu::NEW_VAL ) // Uncomment to use only data syncronized with the video frames
//         {
//             // ----> Data info to be displayed
//             std::stringstream timestamp;
//             std::stringstream accel;
//             std::stringstream gyro;

//             timestamp << std::fixed << std::setprecision(9) << "IMU timestamp:   " << static_cast<double>(imuData.timestamp)/1e9<< " sec" ;
//             if(last_imu_ts!=0)
//                 timestamp << std::fixed << std::setprecision(1)  << " [" << 1e9/static_cast<float>(imuData.timestamp-last_imu_ts) << " Hz]";
//             last_imu_ts = imuData.timestamp;

//             accel << std::fixed << std::showpos << std::setprecision(4) << " * Accel: " << imuData.aX << " " << imuData.aY << " " << imuData.aZ << " [m/s^2]";
//             gyro << std::fixed << std::showpos << std::setprecision(4) << " * Gyro: " << imuData.gX << " " << imuData.gY << " " << imuData.gZ << " [deg/s]";
//             // <---- Data info to be displayed

//             // Mutex to not overwrite data while diplaying them
//             imuMutex.lock();

//             imuTsStr = timestamp.str();
//             imuAccelStr = accel.str();
//             imuGyroStr = gyro.str();

//             // ----> Timestamp of the synchronized data
//             if(imuData.sync)
//             {
//                 mcu_sync_ts = imuData.timestamp;
//             }
//             // <---- Timestamp of the synchronized data

//             imuMutex.unlock();
//         }
//         // <---- Get IMU data
//     }
// }


void saveVideoThreadFunc(std::string time)
{
	// Flag to stop the thread
        saveThreadStop = false;
	int i = 0;
	while(!saveThreadStop)
	{

		imageMutex.lock();
        	if(!imageQueue.empty())
        	{
			std::cout << "Saving image: "+ std::to_string(i) << std::endl;
			cv::imwrite(save_path + time + "/images/image_"+  std::to_string(i) + ".png", imageQueue.front());
            		// Display image
	    		// cv::imshow( "Stream RGB", imageQueue.front());
            		imageQueue.pop();
            		i++;
        	}
        	imageMutex.unlock();
	}
}
void getVideoThreadFunc(sl_oc::video::VideoCapture* videoCap, sl_oc::video::RESOLUTION resolution)
{
    // Flag to stop the thread
    videoThreadStop = false;

    int w,h;
    videoCap->getFrameSize(w,h);

    cv::Size display_resolution(1024, 576);

    switch(resolution)
    {
    default:
    case sl_oc::video::RESOLUTION::VGA:
        display_resolution.width = w;
        display_resolution.height = h;
        break;
    case sl_oc::video::RESOLUTION::HD720:
        display_resolution.width = w*0.6;
        display_resolution.height = h*0.6;
        break;
    case sl_oc::video::RESOLUTION::HD1080:
    case sl_oc::video::RESOLUTION::HD2K:
        display_resolution.width = w*0.4;
        display_resolution.height = h*0.4;
        break;
    }

    int h_data = 70;
    cv::Mat frameDisplay(display_resolution.height + h_data, display_resolution.width,CV_8UC3, cv::Scalar(0,0,0));
    cv::Mat frameData = frameDisplay(cv::Rect(0,0, display_resolution.width, h_data));
    cv::Mat frameBGRDisplay = frameDisplay(cv::Rect(0,h_data, display_resolution.width, display_resolution.height));
    cv::Mat frameBGR(h, w, CV_8UC3, cv::Scalar(0,0,0));

    // Infinite data grabbing loop
    uint64_t last_timestamp = 0;
    float frame_fps=0;
    int i = 0;
    while(!videoThreadStop)
    {
        // ----> Get Video frame
        // Get last available frame
        const sl_oc::video::Frame frame = videoCap->getLastFrame(1);

        // If the frame is valid we can update it
        std::stringstream videoTs;
        if(frame.data!=nullptr && frame.timestamp!=last_timestamp)
        {

		std::cout << "Capturing image: "+ std::to_string(i) << std::endl;
	        if (i%stride == 0) {
            // ************* JM ***************
                int exposure = exposures[(i/stride) % exposures.size()];
                videoCap->setExposure(sl_oc::video::CAM_SENS_POS::RIGHT, exposure);
                // videoCap.setExposure(sl_oc::video::CAM_SENS_POS::LEFT, exposure);
            }
            // ********************************
            frame_fps = 1e9/static_cast<float>(frame.timestamp-last_timestamp);
            last_timestamp = frame.timestamp;

            // ----> Conversion from YUV 4:2:2 to BGR for visualization
            cv::Mat frameYUV( frame.height, frame.width, CV_8UC2, frame.data);
            cv::cvtColor(frameYUV,frameBGR, cv::COLOR_YUV2BGR_YUYV);
            // <---- Conversion from YUV 4:2:2 to BGR for visualization
	    
	    std::cout << std::to_string(frame_fps) << std::endl;

	    // RECORDING
            imageMutex.lock();
            imageQueue.push(frameBGR);
            imageMutex.unlock();
	    i++;
        }
        // <---- Get Video frame
    }
}
