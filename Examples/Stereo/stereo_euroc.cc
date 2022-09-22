/**
 * @file stereo.cc
 * @author Yanwei Du (yanwei.du@gatech.edu)
 * @brief None
 * @version 0.1
 * @date 09-19-2022
 * @copyright Copyright (c) 2022
 */

#include <chrono>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>

#include <opencv2/core/core.hpp>

#include <System.h>

using namespace std;
void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps);

int main(int argc, char **argv)
{
    if(argc != 11)
    {
        cerr << endl << "Usage: ./stereo_euroc path_to_vocabulary path_to_settings budget_per_frame do_rectify enable_viewer path_to_image_folder path_to_times_file path_traj path_to_map speed_option" << endl;
        return 1;
    }
    const string path_traj(argv[8]);
    const double speed = stod(argv[10]);
    const bool enable_viwer = stoi(argv[5]);
    const bool do_rectify = stoi(argv[4]);

    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimeStamp;
    {
        const string pathSeq(argv[6]);
        const string pathCam0 = pathSeq + "/mav0/cam0/data";
        const string pathCam1 = pathSeq + "/mav0/cam1/data";
        LoadImages(pathCam0, pathCam1, string(argv[7]), vstrImageLeft, vstrImageRight, vTimeStamp);
    }

    if(vstrImageLeft.empty() || vstrImageRight.empty())
    {
        cerr << "ERROR: No images in provided path." << endl;
        return 1;
    }

    if(vstrImageLeft.size()!=vstrImageRight.size())
    {
        cerr << "ERROR: Different number of left and right images." << endl;
        return 1;
    }

    // Read rectification parameters
    cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "ERROR: Wrong path to settings" << endl;
        return -1;
    }

    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
    fsSettings["LEFT.K"] >> K_l;
    fsSettings["RIGHT.K"] >> K_r;

    fsSettings["LEFT.P"] >> P_l;
    fsSettings["RIGHT.P"] >> P_r;

    fsSettings["LEFT.R"] >> R_l;
    fsSettings["RIGHT.R"] >> R_r;

    fsSettings["LEFT.D"] >> D_l;
    fsSettings["RIGHT.D"] >> D_r;

    int rows_l = fsSettings["LEFT.height"];
    int cols_l = fsSettings["LEFT.width"];
    int rows_r = fsSettings["RIGHT.height"];
    int cols_r = fsSettings["RIGHT.width"];

    if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
    {
        cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
        return -1;
    }

    cv::Mat M1l,M2l,M1r,M2r;
    cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
    cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);


    const int nImages = vstrImageLeft.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,enable_viwer);

    SLAM.SetConstrPerFrame(stoi(argv[3]));

#ifdef REALTIME_TRAJ_LOGGING
    std::string fNameRealTimeTrack = std::string(argv[8]) + "_AllFrameTrajectory.txt";
    std::cout << std::endl << "Saving AllFrame Trajectory to AllFrameTrajectory.txt" << std::endl;
    SLAM.SetRealTimeFileStream(fNameRealTimeTrack);
#endif

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.reserve(nImages);
    vector<pair<double, double> > vStampedTimesTrack;

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat imLeft, imRight, imLeftRect, imRightRect;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[ni],CV_LOAD_IMAGE_GRAYSCALE);
        imRight = cv::imread(vstrImageRight[ni],CV_LOAD_IMAGE_GRAYSCALE);

        if(imLeft.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageLeft[ni]) << endl;
            return 1;
        }

        if(imRight.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageRight[ni]) << endl;
            return 1;
        }

        if (do_rectify)
        {
            cv::remap(imLeft,imLeftRect,M1l,M2l,cv::INTER_LINEAR);
            cv::remap(imRight,imRightRect,M1r,M2r,cv::INTER_LINEAR);
        }

        double tframe = vTimeStamp[ni];


        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // Pass the images to the SLAM system
        if (do_rectify)
        {
            SLAM.TrackStereo(imLeftRect, imRightRect, tframe);
        }
        else
        {
            SLAM.TrackStereo(imLeft, imRight,tframe);
        }

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        std::cout << "Pose Tracking Latency: " << ttrack << " sec" << std::endl;

        vTimesTrack.emplace_back(ttrack);
        vStampedTimesTrack.emplace_back(tframe, ttrack);

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimeStamp[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimeStamp[ni-1];

        // consider speed
        T = T / speed;

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
        else // catch up with images
        {
            while (ni < nImages - 1 && 
                    (vTimeStamp[ni+1] - tframe) / speed < ttrack)
            {
                ni++;
            }
        }
    }

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    int proccIm = vTimesTrack.size();
    for(int ni=0; ni<proccIm; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "processed images: " << proccIm << endl;
    cout << "median tracking time: " << vTimesTrack[proccIm/2] << endl;
    cout << "mean tracking time: " << totaltime/proccIm << endl;
    cout << "min tracking time: " << vTimesTrack.front() << endl;
    cout << "max tracking time: " << vTimesTrack.back() << endl;

    // save to stats
    {
        std::ofstream myfile(path_traj + "_stats.txt");
        myfile << std::setprecision(6) << nImages << " "
               << proccIm << " "
               << totaltime / proccIm << " "
               << vTimesTrack[proccIm/2] << " "
               << vTimesTrack.front() << " "
               << vTimesTrack.back() << " ";
        myfile.close();

        myfile.open(path_traj + "_timeLog.txt");
        myfile << std::setprecision(20);
        for (const auto& m : vStampedTimesTrack)
        {
            myfile << m.first << " " << m.second << "\n";
        }
        myfile.close();
    }

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM( std::string(argv[8]) + "_KeyFrameTrajectory.txt" );
    SLAM.SaveTrackingLog( std::string(argv[8]) + "_Log.txt" );
#ifdef LOCAL_BA_TIME_LOGGING
    SLAM.SaveMappingLog( std::string(argv[8]) + "_Log_Mapping.txt" );
#endif
    //
    // SLAM.SaveLmkLog( std::string(argv[8]) + "_Log_lmk.txt" );

/*
#ifdef ENABLE_MAP_IO

// #ifdef MAP_PUBLISH
// publish map points
//  for (size_t i=0; i<10; ++i)
//   igb.mpMapPub->Refresh();
// #endif
  
  SLAM.SaveMap(std::string(argv[9]));
  // SLAM.SaveMap(std::string(argv[8]) + "_Map/");
  
#endif
*/

    std::cout << "Finished saving!" << std::endl;

    // Stop all threads
    SLAM.Shutdown();
    cout << "stereo_euroc: done with SLAM Shutdown!" << endl;

    return 0;
}

void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImageLeft.reserve(5000);
    vstrImageRight.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImageLeft.push_back(strPathLeft + "/" + ss.str() + ".png");
            vstrImageRight.push_back(strPathRight + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);

        }
    }
}
