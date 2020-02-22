/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>
#include<librealsense2/rs.hpp>
#include<opencv2/opencv.hpp>

#include<System.h>

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    if (argc != 3) {
        cerr << endl << "Usage ./agv_orb path_to_vocabulary path_to_settings" << endl;
    }
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;

    cout << endl << "-------" << endl;
    cout << endl << "Creating intel realsense pipeline." << endl;

    rs2::pipeline pipe;
    // rs2::config cfg;
    // cfg.enable_stream(RS2_STREAM_ANY, 640, 480, RS2_FORMAT_, 10);

    cout << endl << "Opening camera." << endl;

    pipe.start();

    cout << endl << "Flushing stream." << endl;
    rs2::frameset frames;
    for (int i = 0; i < 10; i++) {
        frames = pipe.wait_for_frames();
    }

    // Main loop
    cv::Mat imRGB, imD;
    while(1)
    {
        frames = pipe.wait_for_frames();
        
        rs2::frame color_frame = frames.get_color_frame();
        rs2::frame depth_frame = frames.get_depth_frame();

        cv::Mat imRGB(Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat imD(Size(640, 480), CV_8UC3, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif
        // double timestamp = frames.get_timestamp();
        auto timeStamp = chrono::steady_clock::now();
        chrono::duration<double> timestamp = chrono::duration_cast<chrono::duration<double>(timeStamp);

        cout << timestamp.count() << endl;
        // Pass the image to the SLAM system
        SLAM.TrackRGBD(imRGB,imD,timestamp.count());

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");   

    return 0;
}
