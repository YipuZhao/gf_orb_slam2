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

#include <unistd.h>

#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>
#include <time.h>
#include <random>

//
#include <boost/filesystem.hpp>


bool has_suffix(const std::string &str, const std::string &suffix) {
    std::size_t index = str.find(suffix, str.size() - suffix.size());
    return (index != std::string::npos);
}

namespace ORB_SLAM2
{

System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
               const bool bUseViewer):mSensor(sensor), mpViewer(static_cast<Viewer*>(NULL)), mbReset(false),mbActivateLocalizationMode(false),
    mbDeactivateLocalizationMode(false)
{
    // Output welcome message
    cout << endl <<
            "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
            "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
            "This is free software, and you are welcome to redistribute it" << endl <<
            "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;

    //Check settings file
    cout << "Try loading yaml from " << strSettingsFile.c_str() << endl;
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "Failed to open settings file at: " << strSettingsFile << endl;
        exit(-1);
    }


    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    clock_t tStart = clock();

    mpVocabulary = new ORBVocabulary();
    // bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    bool bVocLoad = false; // chose loading method based on file extension
    if (has_suffix(strVocFile, ".txt"))
        bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    else
        bVocLoad = mpVocabulary->loadFromBinaryFile(strVocFile);

    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Failed to open at: " << strVocFile << endl;
        exit(-1);
    }
    // cout << "Vocabulary loaded!" << endl << endl;
    printf("Vocabulary loaded in %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);

    //Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    //Create the Map
    mpMap = new Map();

    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpMap);
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

    mpHashHandler = new HASHING::MultiIndexHashing(256, NUM_TOTAL_HASHTABLES);

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpMap, mSensor==MONOCULAR);
    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run,mpLocalMapper);

    //Initialize the Loop Closing thread and launch
    mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR);
    mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

    //Initialize the Viewer thread and launch
    if(bUseViewer)
    {
        mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile);
        mptViewer = new thread(&Viewer::Run, mpViewer);
        mpTracker->SetViewer(mpViewer);
    }

    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);
    mpTracker->SetHashHandler(mpHashHandler);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);
    mpLocalMapper->SetHashHandler(mpHashHandler);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);

    mFrameLossTrack = 0;


#ifdef ORB2_BASELINE
    std::cout << "System: ORB2 baseline mode enabled!" << std::endl;
#elif defined GF_MODE
    std::cout << "System: GF mode enabled!" << std::endl;
#elif defined GF_SW_MODE
    std::cout << "System: GF+SW mode enabled!" << std::endl;
#elif defined GF_CV_MODE
    std::cout << "System: GF+CV mode enabled!" << std::endl;
#elif defined GF_GG_MODE
    std::cout << "System: GF+CV mode enabled!" << std::endl;
#else
    std::cout << "System: Unknown mode enabled! Be caution." << std::endl;
#endif
}

cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
{
    if(mSensor!=STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
        }
    }

    cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft,imRight,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;


    //
    if (mTrackingState == Tracking::LOST) {
        mFrameLossTrack ++;
        std::cout << "Track Lost!" << std::endl;
    }
    else {
        // reset the frame counter
        mFrameLossTrack = 0;
    }

    // terminate the whole pipeline if certain amount of frames being loss track
    if (mFrameLossTrack > MAX_FRAME_LOSS_DURATION * mpTracker->camera_fps) {
        std::cout << "Too many frames loss track, terminate the SLAM pipeline!" << std::endl;
        Shutdown();
    }


    return Tcw;
}

cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
{
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
        }
    }

    cv::Mat Tcw = mpTracker->GrabImageRGBD(im,depthmap,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    //
    if (mTrackingState == Tracking::LOST) {
        mFrameLossTrack ++;
        std::cout << "Track Lost!" << std::endl;
    }
    else {
        // reset the frame counter
        mFrameLossTrack = 0;
    }

    // terminate the whole pipeline if certain amount of frames being loss track
    if (mFrameLossTrack > MAX_FRAME_LOSS_DURATION * mpTracker->camera_fps) {
        std::cout << "Too many frames loss track, terminate the SLAM pipeline!" << std::endl;
        Shutdown();
    }

    return Tcw;
}

cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
    if(mSensor!=MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
        }
    }

    cv::Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;


    //
    if (mTrackingState == Tracking::LOST) {
        mFrameLossTrack ++;
        std::cout << "Track Lost!" << std::endl;
    }
    else {
        // reset the frame counter
        mFrameLossTrack = 0;
    }

    // terminate the whole pipeline if certain amount of frames being loss track
    if (mFrameLossTrack > MAX_FRAME_LOSS_DURATION * mpTracker->camera_fps) {
        std::cout << "Too many frames loss track, terminate the SLAM pipeline!" << std::endl;
        Shutdown();
    }

    return Tcw;
}

void System::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

bool System::MapChanged()
{
    static int n=0;
    int curn = mpMap->GetLastBigChangeIdx();
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::Shutdown()
{
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    if(mpViewer)
    {
        mpViewer->RequestFinish();
        while(!mpViewer->isFinished())
            usleep(5000);

        //        // This code is from jkwkch, from
        //        //   https://github.com/raulmur/ORB_SLAM2/issues/547
        //        delete mpViewer;
        //        mpViewer = static_cast<Viewer*>(NULL);
    }
    //    cout << "At the stage 1 of system Shutdown!" << endl;

    // Wait until all thread have effectively stopped
    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
    {
        usleep(5000);
    }
    //    cout << "At the stage 2 of system Shutdown!" << endl;

    // NOTE
    // for some reason the shutdown func gets blocked at this line;
    // disabled the text change for smooth termination & logging
    //    if(mpViewer)
    //        pangolin::BindToContext("ORB-SLAM2: Map Viewer");

    cout << "At the end of system Shutdown!" << endl;
}

void System::SetRealTimeFileStream(const string &track_fname, const string &BA_fname)
{
    mpTracker->SetRealTimeFileStream(track_fname);
    std::cout << "mpTracker real time output to " << track_fname << std::endl;
    //
#ifdef LOGGING_KF_LIST
    mpLocalMapper->SetRealTimeFileStream(BA_fname);
    std::cout << "mpLocalMapper real time output to " << BA_fname << std::endl;
#endif
}

void System::SetRealTimeFileStream(const string &filename)
{
    mpTracker->SetRealTimeFileStream(filename);
    std::cout << "mpTracker real time output to " << filename << std::endl;
}


void System::SetBudgetPerFrame(const double budget_per_frame)
{
    mpTracker->time_track_budget = budget_per_frame;
    std::cout << "mpTracker budget_per_frame adjusted to " << mpTracker->time_track_budget << std::endl;

#ifdef ENABLE_ANTICIPATION_IN_GRAPH
    mpTracker->mVFrameInteval = VIRTUAL_FRAME_STEP * mpTracker->time_track_budget;
    std::cout << "mpTracker mVFrameInteval adjusted to " << mpTracker->mVFrameInteval << std::endl;
#endif
}

void System::SetConstrPerFrame(const size_t constr_per_frame)
{
    mpTracker->num_good_constr_predef = constr_per_frame;
#ifdef ORB_SLAM_BASELINE
    // baseline only
    // re-create orb extractor with the defined budget per frame
    mpTracker->updateORBExtractor();
#endif
    std::cout << "mpTracker constraint adjusted to " << mpTracker->num_good_constr_predef << std::endl;
}


void System::GrabAllLmkLog() {

    const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    if(vpMPs.empty())
        return;

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        MapPoint * pMP = vpMPs[i];
        if(!pMP || pMP->isBad())
            continue;
        //
        const map<KeyFrame*,size_t> observations = pMP->GetObservations();
        size_t N = observations.size();
        if (N == 0)
            continue ;
        size_t id = pMP->mnId;
        size_t life = observations.rbegin()->first->mnFrameId - observations.begin()->first->mnFrameId + 1;
        logLmkLife.push_back(LmkLog(id, life));
    }
}

void System::SaveLmkLog(const std::string &filename) {

    GrabAllLmkLog();

    size_t N = logLmkLife.size();
    std::cout << std::endl << "Saving " << N << " records to lmk log file " << filename << " ..." << std::endl;

    std::ofstream fLmkLog;
    fLmkLog.open(filename.c_str());
    fLmkLog << std::fixed;
    fLmkLog << "#id life" << std::endl;
    for(size_t i=0; i<N; i++)
    {
        fLmkLog << std::setprecision(0)
                << logLmkLife[i].id << " "
                << logLmkLife[i].life << std::endl;
    }
    fLmkLog.close();

    std::cout << "Finished saving lmk log! " << std::endl;
}

void System::SaveTrackingLog(const string &filename) {

    cout << endl << "Saving tracking log to " << filename << " ..." << endl;

    ofstream fFrameTimeLog;
    fFrameTimeLog.open(filename.c_str());
    fFrameTimeLog << fixed;
    fFrameTimeLog << "#frame_time_stamp time_ORB_extraction time_track_motion time_track_frame time_track_map time_match ..." << std::endl;
    for(size_t i=0; i<mpTracker->mFrameTimeLog.size(); i++)
    {

        fFrameTimeLog << setprecision(6)
                      << mpTracker->mFrameTimeLog[i].frame_time_stamp << " "
                      << mpTracker->mFrameTimeLog[i].time_rectification + mpTracker->mFrameTimeLog[i].time_ORB_extraction << " "
                      << mpTracker->mFrameTimeLog[i].time_track_motion << " "
                      << mpTracker->mFrameTimeLog[i].time_track_frame << " "
                      << mpTracker->mFrameTimeLog[i].time_track_map << " "
                      << mpTracker->mFrameTimeLog[i].time_match << " "
                      << mpTracker->mFrameTimeLog[i].time_select << " "
                      << mpTracker->mFrameTimeLog[i].time_optim << " "
                      << mpTracker->mFrameTimeLog[i].time_stereo_motion << " "
                      << mpTracker->mFrameTimeLog[i].time_stereo_frame << " "
                      << mpTracker->mFrameTimeLog[i].time_stereo_map << " "
                      << mpTracker->mFrameTimeLog[i].time_stereo_post << " "
                      << mpTracker->mFrameTimeLog[i].time_post_proc << " "
                      << mpTracker->mFrameTimeLog[i].time_mat_online << " "
                      << mpTracker->mFrameTimeLog[i].time_hash_insert << " "
                      << mpTracker->mFrameTimeLog[i].time_hash_query << " "
                      << setprecision(0)
                      << mpTracker->mFrameTimeLog[i].lmk_num_motion << " "
                      << mpTracker->mFrameTimeLog[i].lmk_num_frame << " "
                      << mpTracker->mFrameTimeLog[i].lmk_num_map << " "
                      << mpTracker->mFrameTimeLog[i].lmk_localmap_comb << " "
                      << mpTracker->mFrameTimeLog[i].lmk_num_BA << " "
                      << mpTracker->mFrameTimeLog[i].lmk_hash_dynamics << std::endl;
    }

    fFrameTimeLog.close();
}


void System::SaveMappingLog(const string &filename) {

    cout << endl << "Saving mapping log to " << filename << " ..." << endl;

    ofstream fBATimeLog;
    fBATimeLog.open(filename.c_str());
    fBATimeLog << fixed;
    fBATimeLog << "#frame_time_stamp time_proc_new_keyframe time_culling time_tri_new_map_point time_srh_more_neighbor time_local_BA num_fixed_KF num_free_KF num_Point" << std::endl;
    for(size_t i=0; i<mpLocalMapper->mvBATimeLog.size(); i++)
    {
        //        fBATimeLog << setprecision(6)
        //                   << mpLocalMapper->mBATimeLog[i].frame_time_stamp << " "
        //                   << mpLocalMapper->mBATimeLog[i].time_proc_new_keyframe << " "
        //                   << mpLocalMapper->mBATimeLog[i].time_culling << " "
        //                   << mpLocalMapper->mBATimeLog[i].time_tri_new_map_point << " "
        //                   << mpLocalMapper->mBATimeLog[i].time_srh_more_neighbor << " "
        //                   << mpLocalMapper->mBATimeLog[i].time_local_BA << " "
        //                   << setprecision(0)
        //                   << mpLocalMapper->mBATimeLog[i].num_fixed_KF << " "
        //                   << mpLocalMapper->mBATimeLog[i].num_free_KF << " "
        //                   << mpLocalMapper->mBATimeLog[i].num_Point << std::endl;

        fBATimeLog << setprecision(6)
                   << mpLocalMapper->mvBATimeLog[i].frame_time_stamp << " "
                   << mpLocalMapper->mvBATimeLog[i].time_proc_new_keyframe << " "
                   << mpLocalMapper->mvBATimeLog[i].time_culling << " "
                   << mpLocalMapper->mvBATimeLog[i].time_tri_new_map_point << " "
                   << mpLocalMapper->mvBATimeLog[i].time_srh_more_neighbor << " "
                   << mpLocalMapper->mvBATimeLog[i].time_local_BA << " "
                      //
                   << mpLocalMapper->mvBATimeLog[i].time_gg_prediction << " "
                   << mpLocalMapper->mvBATimeLog[i].time_gg_insert_vertex << " "
                   << mpLocalMapper->mvBATimeLog[i].time_gg_jacobian << " "
                   << mpLocalMapper->mvBATimeLog[i].time_gg_rnd_query << " "
                   << mpLocalMapper->mvBATimeLog[i].time_gg_schur << " "
                   << mpLocalMapper->mvBATimeLog[i].time_gg_permute << " "
                   << mpLocalMapper->mvBATimeLog[i].time_gg_cholesky << " "
                   << mpLocalMapper->mvBATimeLog[i].time_gg_postproc << " "
                   << mpLocalMapper->mvBATimeLog[i].time_gg_optimization << " "
                      // add 1 more column for budget
                   << mpLocalMapper->mvBATimeLog[i].time_gg_lba_budget << " "
                   << setprecision(0)
                   << mpLocalMapper->mvBATimeLog[i].num_fixed_KF << " "
                   << mpLocalMapper->mvBATimeLog[i].num_free_KF << " "
                   << mpLocalMapper->mvBATimeLog[i].num_Point << std::endl;
    }
    fBATimeLog.close();
}

void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::idComp);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = Converter::toQuaternion(Rwc);

        f << setprecision(6) << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::idComp);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

        // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::idComp);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM2::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
            //  cout << "bad parent" << endl;
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

int System::GetTrackingState()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

vector<MapPoint*> System::GetTrackedMapPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

static void FindAllFilesInDir( const std::string & dir, const std::string & ext,
                               std::list<std::string> * filename_list,
                               size_t & filename_maxlen ) {

    // get a sorted list of files in the img directories
    boost::filesystem::path files_dir(dir.c_str());
    if (!boost::filesystem::exists(files_dir))
    {
        std::cout << std::endl << "Input directory does not exist: \t" << files_dir << std::endl;
        return ;
    }

    // get all files in the img directories
    filename_maxlen = 0;
    filename_list->clear();
    boost::filesystem::directory_iterator end_itr;
    for (boost::filesystem::directory_iterator file(files_dir); file != end_itr; ++file)
    {
        boost::filesystem::path filename_path = file->path().filename();
        if (boost::filesystem::is_regular_file(file->status()) &&
                filename_path.extension() == ext.c_str() )
        {
            std::string filename(filename_path.string());
            filename_list->push_back(filename);
            filename_maxlen = max(filename_maxlen, filename.length());
            //            cout << filename << endl;
        }
    }
    //
    // NOTE
    // the following line is mandatory! thought most of the entries are listed in order, there
    // are some mis-aligned ones exist, which have to be fixed with the explicit sorting !!!
    //
    filename_list->sort();

    std::cout << std::endl << "Found " << filename_list->size() << " files under the given dir with extension " << ext << std::endl;
}

void System::ForceRelocTracker() {
    mpTracker->ForceReloc();
}

void System::ForceInitTracker() {
    mpTracker->ForceInit();
}


#if defined ENABLE_ANTICIPATION_IN_GRAPH || defined PRED_WITH_ODOM
//
void System::LoadOdomPlanned(const std::string &odom_path) {
    ifstream f;
    mpTracker->ResetOdomBuffer();

    f.open(odom_path.c_str());
    if (f.is_open()) {
        size_t count = 0;
        std::string line;
        while (std::getline(f, line))
        {
            //            cout << line << endl;
            std::istringstream iss(line);
            double time_stamp, tx, ty, tz, qw, qx, qy, qz;
            if (!(iss >> time_stamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw))
                break; // error

            // process
            //            cv::Mat R = pKF->GetRotation().t();
            //            vector<float> q = Converter::toQuaternion(R);
            //            cv::Mat t = pKF->GetCameraCenter();
            //            f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
            //              << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
            cv::Mat Tcw, Twc;
            Eigen::Quaterniond quat(qw, qx, qy, qz);
            Eigen::Vector3d tran(tx, ty, tz);

            // add noise to ground truth odometry
#ifdef ENABLE_PERTURBATION_TO_ODOM

            //            std::cout << "Before perturbation: " << std::endl;
            //            std::cout << "tx = " << tran(0) << ", ty = " << tran(1) << ", tz = " << tran(2) << std::endl;
            //            std::cout << "qw = " << quat.w() << ", qx = " << quat.x() << ", qy = " << quat.y() << ", qz = " << quat.z() << std::endl;

            std::random_device rd; // obtain a random number from hardware
            std::mt19937 eng(rd()); // seed the generator
            std::uniform_int_distribution<> distr(0, 100); // define the range

            double ex = double(distr(eng)) * ANTICIPATION_NOISE_ROTA_STD / 100.0f;
            double ey = double(distr(eng)) * ANTICIPATION_NOISE_ROTA_STD / 100.0f;
            double ez = double(distr(eng)) * ANTICIPATION_NOISE_ROTA_STD / 100.0f;
            double enorm = sqrt(ex*ex + ey*ey + ez*ez);
            Eigen::AngleAxis<double> eang(enorm, Eigen::Vector3d(ex/enorm, ey/enorm, ez/enorm));
            Eigen::Quaterniond qErr( eang );
            //            Eigen::Quaterniond qErr(sqrt(1 - ex*ex - ey*ey - ez*ez), ex, ey, ez);
            Eigen::Vector3d tErr( double(distr(eng)) * ANTICIPATION_NOISE_TRAN_STD / 100.0f,
                                  double(distr(eng)) * ANTICIPATION_NOISE_TRAN_STD / 100.0f,
                                  double(distr(eng)) * ANTICIPATION_NOISE_TRAN_STD / 100.0f );
            //
            quat *= qErr;
            quat.normalize();
            tran += tErr;

            //            std::cout << "After perturbation: " << std::endl;
            //            std::cout << "tx = " << tran(0) << ", ty = " << tran(1) << ", tz = " << tran(2) << std::endl;
            //            std::cout << "qw = " << quat.w() << ", qx = " << quat.x() << ", qy = " << quat.y() << ", qz = " << quat.z() << std::endl;
#endif

//            g2o::SE3Quat g_wc = g2o::SE3Quat(quat, tran);
//            Twc = Converter::toCvMat(g_wc);
//            //
//            //            Tcw = Twc.inv();
//            cv::Mat Rcw = Twc.rowRange(0, 3).colRange(0, 3).t();
//            cv::Mat tcw = -Rcw * Twc.rowRange(0, 3).col(3);
//            Tcw = cv::Mat::eye(4, 4, CV_32F);
//            Rcw.copyTo(Tcw.rowRange(0, 3).colRange(0, 3));
//            tcw.copyTo(Tcw.rowRange(0, 3).col(3));

//            mpTracker->AddOdomPlanned(time_stamp, Tcw);

            // NOTE
            // the t & q vector inserted here are for Twc
            mpTracker->BufferingOdom(time_stamp, tran(0), tran(1), tran(2),
                                     quat.w(), quat.x(), quat.y(), quat.z());
            count ++;
        }
        f.close();
        std::cout << "func LoadOdomPlanned: " << count << " records loaded!" << std::endl;
    }
    else {
        std::cout << "func LoadOdomPlanned: Unable to open file " << odom_path << std::endl;
    }
    //    f.close();
    //    cout << endl
    //         << "planned trajectory loaded!" << endl;
}

#endif

//
// NOTE
// Unless you are really confident that there won't be any invalid query to the hash map,
// avoid using the shortcut map[key]!
// In the presence of invalid key, this shortcut will insert default object to the hash map,
// therefore leading to unpredicted query results in the future!
// Stick to the auto it & find & end check process for safety!!!
//
#ifdef ENABLE_MAP_IO
// Load & Save Function
void System::LoadMap(const std::string & map_path) {

    //loading KeyFrames..
    cout<<"loading KeyFrames.."<<endl;
    vector<ORB_SLAM2::KeyFrame*> vpKFs;
    vector<vector<double> > mvMapPointsVector;
    vector<vector<double> > mConnectedKeyFrameWeights_firstVector;
    vector<vector<int> > mConnectedKeyFrameWeights_secondVector;
    vector<vector<double> >mvOrderedConnectedKeyFramesVector;
    vector<double> mParentVector;
    vector<vector<double> > msChildrensVector;
    vector<vector<double> > msLoopEdgesVector;
    std::unordered_map<unsigned long, ORB_SLAM2::KeyFrame*> map_ID_2_KF;
    map_ID_2_KF.clear();
    std::unordered_map<unsigned long, ORB_SLAM2::MapPoint*> map_ID_2_Pt;
    map_ID_2_Pt.clear();

    max_KF_Id = 0;
    std::list<std::string> KF_file_list;
    size_t KF_file_maxlen;
    FindAllFilesInDir(map_path + "/KeyFrames", ".yml", &KF_file_list, KF_file_maxlen);
    //
    for(auto iter = KF_file_list.begin(); iter != KF_file_list.end(); iter ++){

        std::string KFfileName = map_path + "/KeyFrames/" + *iter;

        cv::FileStorage fs(KFfileName.c_str(), cv::FileStorage::READ);
        if(!fs.isOpened()) {
            cout<<"no such file!"<<endl;
            continue ;
        }

        ORB_SLAM2::KeyFrame* pKF = new ORB_SLAM2::KeyFrame(fs, mpMap, mpVocabulary, mpKeyFrameDatabase);

        pKF->mnTrackReferenceForFrame = 0;
        //        pKF->N = N_in_KF;

        vector<double> mvMapPoints;
        fs["mvMapPoints"] >> mvMapPoints;
        //        cout << mvMapPoints.size() << endl;
        //        assert(mvMapPoints.size() == pKF->N);
        pKF->N = mvMapPoints.size();
        mvMapPointsVector.push_back(mvMapPoints);

        vector<double> mConnectedKeyFrameWeights_first;
        vector<int> mConnectedKeyFrameWeights_second;
        fs["mConnectedKeyFrameWeights_first"] >> mConnectedKeyFrameWeights_first;
        fs["mConnectedKeyFrameWeights_second"] >> mConnectedKeyFrameWeights_second;
        mConnectedKeyFrameWeights_firstVector.push_back(mConnectedKeyFrameWeights_first);
        mConnectedKeyFrameWeights_secondVector.push_back(mConnectedKeyFrameWeights_second);

        vector<double> mvOrderedConnectedKeyFrames;
        fs["mvOrderedConnectedKeyFrames"] >> mvOrderedConnectedKeyFrames;
        mvOrderedConnectedKeyFramesVector.push_back(mvOrderedConnectedKeyFrames);

        double mParent;
        fs["mParent"] >> mParent;
        mParentVector.push_back(mParent);
        vector<double> msChildrens;
        fs["msChildrens"] >> msChildrens;
        msChildrensVector.push_back(msChildrens);
        vector<double> msLoopEdges;
        fs["msLoopEdges"] >> msLoopEdges;
        msLoopEdgesVector.push_back(msLoopEdges);
        //        cout << pKF->mnId << endl;

        map_ID_2_KF.insert({pKF->mnId, pKF});
        vpKFs.push_back(pKF);

        fs.release();

        max_KF_Id = max(pKF->mnId, max_KF_Id);
    }//end of creating KeyFrames

    //loading MapPoints..
    cout<<"loading MapPoints.."<<endl;
    vector<ORB_SLAM2::MapPoint*> vpMPs;
    vector<vector<int> > mObservations_firstVector;
    vector<vector<int> > mObservations_secondVector;
    vector<int> mObservations_first;
    vector<int> mObservations_second;
    vector<double> mRefKFVector;

    max_Pt_Id = 0;
    std::list<std::string> MP_file_list;
    size_t MP_file_maxlen;
    FindAllFilesInDir(map_path + "/MapPoints", ".yml", &MP_file_list, MP_file_maxlen);

    //    cout << "map_ID_2_KF.find(1) = " << map_ID_2_KF.find(1)->second << endl;
    for(auto iter = MP_file_list.begin(); iter != MP_file_list.end(); iter ++){

        std::string MPfileName = map_path + "/MapPoints/" + *iter;
        //        cout << *iter << endl;

        cv::FileStorage fs(MPfileName.c_str(), cv::FileStorage::READ);
        if(!fs.isOpened()) {
            cout<<"no such file!"<<endl;
            continue ;
        }

        ORB_SLAM2::MapPoint* pMP = new ORB_SLAM2::MapPoint(fs, mpMap);

        mObservations_first.clear();
        fs["mObservations_first"] >> mObservations_first;
        vector<int> firstVec_tmp(mObservations_first);
        mObservations_firstVector.push_back(firstVec_tmp);
        //
        mObservations_second.clear();
        fs["mObservations_second"] >> mObservations_second;
        vector<int> secondVec_tmp(mObservations_second);
        mObservations_secondVector.push_back(secondVec_tmp);

        //        cout << "mObservations_first: " ;
        //        for (int j=0; j<mObservations_first.size(); ++j)
        //            cout << mObservations_first[j] << ", " ;
        //        cout << endl;

        //        cout << "mObservations_second: " ;
        //        for (int j=0; j<mObservations_second.size(); ++j)
        //            cout << mObservations_second[j] << ", " ;
        //        cout << endl;

        //TODO mpRefKF
        double mRefKF;
        fs["mRefKF"] >> mRefKF;
        mRefKFVector.push_back(mRefKF);

        map_ID_2_Pt.insert({pMP->mnId, pMP});
        vpMPs.push_back(pMP);

        fs.release();

        max_Pt_Id = max(pMP->mnId, max_Pt_Id);
    }//end of creating MapPoints

    //    //loading Reference MapPoints..
    //    cout<<"loading Reference MapPoints.."<<endl;
    //    vector<ORB_SLAM2::MapPoint*> vpRMPs;
    //    vector<vector<int> > mObservations_firstVectorR;//reference
    //    vector<vector<int> > mObservations_secondVectorR;//reference
    //    vector<double> mRefKFVectorR;//reference
    //    std::unordered_map<long unsigned int, ORB_SLAM2::MapPoint*> id2pRMPmap;

    //    std::list<std::string> RM_file_list;
    //    size_t RM_file_maxlen;
    //    FindAllFilesInDir(map_path + "/RMapPoints", ".yml", &RM_file_list, RM_file_maxlen);

    //    for(auto iter = RM_file_list.begin(); iter != RM_file_list.end(); iter ++){
    //        //        string MPfileName = ros::package::getPath("ORB_SLAM")+"/bin/RMapPoints/car_sample/"+"rmappoint_" + boost::to_string(i) + ".yml";
    //        //        std::string MPfileName = map_path + "/RMapPoints/rmappoint_" + boost::to_string(i) + ".yml";
    //        std::string MPfileName = map_path + "/RMapPoints/" + *iter;

    //        cv::FileStorage fs(MPfileName.c_str(), cv::FileStorage::READ);
    //        if(!fs.isOpened()) {
    //            cout<<"no such file!"<<endl;
    //            continue ;
    //        }

    //        ORB_SLAM2::MapPoint* pRMP = new ORB_SLAM2::MapPoint();

    //        mObservations_first.clear();
    //        fs["mObservations_first"] >> mObservations_first;
    //        mObservations_firstVectorR.push_back(mObservations_first);
    //        //
    //        mObservations_second.clear();
    //        fs["mObservations_second"] >> mObservations_second;
    //        mObservations_secondVectorR.push_back(mObservations_second);

    //        double mRefKF;
    //        fs["mRefKF"] >> mRefKF;
    //        mRefKFVectorR.push_back(mRefKF);

    //        id2pRMPmap[pRMP->mnId] = pRMP;
    //        vpRMPs.push_back(pRMP);

    //        fs.release();
    //    }//end of creating Reference MapPoints

    //Assigning KeyFrames..
    cout<<"Assigning KeyFrames.."<<endl;
    //    cout << "map_ID_2_KF.find(1) = " << map_ID_2_KF.find(1)->second << endl;
    for(size_t i = 0; i < vpKFs.size(); i++){
        //        cout << i << endl;
        //loading mvpMapPoints
        vector<double> mvMapPoints = mvMapPointsVector[i];
        for(size_t j = 0; j < mvMapPoints.size(); j++){
            auto it = map_ID_2_Pt.find(mvMapPoints[j]);
            if(mvMapPoints[j] >= 0 && it != map_ID_2_Pt.end())
                //                vpKFs[i]->mvpMapPoints.push_back(id2pMPmap[mvMapPoints[j]]);
                vpKFs[i]->AppendMapPoint(it->second);
            else
                //                vpKFs[i]->mvpMapPoints.push_back(NULL);
                vpKFs[i]->AppendMapPoint(NULL);
        }
        //        cout << "s1" << endl;

        //loading mConnectedKeyFrameWeights
        vector<double> mConnectedKeyFrameWeights_first = mConnectedKeyFrameWeights_firstVector[i];
        vector<int> mConnectedKeyFrameWeights_second = mConnectedKeyFrameWeights_secondVector[i];
        for(size_t j = 0; j < mConnectedKeyFrameWeights_second.size(); j++){
            //            vpKFs[i]->mConnectedKeyFrameWeights[id2pKFmap[mConnectedKeyFrameWeights_first[j]]] = mConnectedKeyFrameWeights_second[j];
            if (mConnectedKeyFrameWeights_first[j] >= 0) {
                auto ait = map_ID_2_KF.find(mConnectedKeyFrameWeights_first[j]);
                if (ait != map_ID_2_KF.end())
                    vpKFs[i]->AddConnection(ait->second, mConnectedKeyFrameWeights_second[j]);
                //  vpKFs[i]->AddConnection(map_ID_2_KF[mConnectedKeyFrameWeights_first[j]], mConnectedKeyFrameWeights_second[j]);
            }
        }
        //        cout << "s2" << endl;

        //loading mvpOrderedConnectedKeyFrames
        vector<double> mvOrderedConnectedKeyFrames =  mvOrderedConnectedKeyFramesVector[i];
        for(size_t j = 0; j < mvOrderedConnectedKeyFrames.size(); j++){
            //            vpKFs[i]->mvpOrderedConnectedKeyFrames.push_back(id2pKFmap[mvOrderedConnectedKeyFrames[j]]);
            if (mvOrderedConnectedKeyFrames[j] >= 0) {
                auto ait = map_ID_2_KF.find(mvOrderedConnectedKeyFrames[j]);
                if (ait != map_ID_2_KF.end())
                    vpKFs[i]->AddCovisibleKeyFrames(ait->second);
                //                vpKFs[i]->AddCovisibleKeyFrames(map_ID_2_KF[mvOrderedConnectedKeyFrames[j]]);
            }
        }
        //        cout << "s3" << endl;

        //loading mParent
        double mParent = mParentVector[i];
        //        vpKFs[i]->mpParent = id2pKFmap[mParent];
        cout << "mParent = " << mParent << "; ";
        if (mParent >= 0) {
            auto iter = map_ID_2_KF.find((unsigned long)(mParent));
            if (iter != map_ID_2_KF.end() && iter->second != NULL) {
                vpKFs[i]->ChangeParent(iter->second);
                cout << "iter->second->mnId = " << iter->second->mnId << "; ";
                cout << "vpKFs[i]->mpParent->mnId = " << vpKFs[i]->GetParent()->mnId << "; ";
                cout << "vpKFs[i]->mpParent->mnTrackReferenceForFrame = " << vpKFs[i]->GetParent()->mnTrackReferenceForFrame << "; ";
            }
        }
        cout << endl;
        //        cout << "s3.1" << endl;

        //loading msChildrens
        vector<double> msChildrens = msChildrensVector[i];
        for(size_t j = 0; j < msChildrens.size(); j++){
            //            vpKFs[i]->mspChildrens.insert(id2pKFmap[msChildrens[j]]);
            if (msChildrens[j] >= 0) {
                auto cit = map_ID_2_KF.find(msChildrens[j]);
                if (cit != map_ID_2_KF.end())
                    vpKFs[i]->AddChild(cit->second);
                //                vpKFs[i]->AddChild(map_ID_2_KF[msChildrens[j]]);
            }
        }
        //        cout << "s3.2" << endl;

        //loading msLoopEdges
        vector<double> msLoopEdges = msLoopEdgesVector[i];
        for(size_t j = 0; j < msLoopEdges.size(); j++){
            //            vpKFs[i]->mspLoopEdges.insert(id2pKFmap[msLoopEdges[j]]);
            if (msLoopEdges[j] >= 0) {
                auto lit = map_ID_2_KF.find(msLoopEdges[j]);
                if (lit != map_ID_2_KF.end())
                    vpKFs[i]->AddLoopEdge(lit->second);
                //                vpKFs[i]->AddLoopEdge(map_ID_2_KF[msLoopEdges[j]]);
            }
        }
        //        cout << "s4" << endl;
    }

    //Assigning MapPoints..
    cout<<"Assigning MapPoints.."<<endl;
    //    cout << "map_ID_2_KF.find(1) = " << map_ID_2_KF.find(1)->second << endl;
    for(size_t i = 0; i < vpMPs.size(); i++){
        //loading mObservations
        vector<int> mObservations_first = mObservations_firstVector[i];
        vector<int> mObservations_second = mObservations_secondVector[i];
        //        cout << "Map Point " << i << "; " << mObservations_first.size() << " vs. " << mObservations_second.size() << endl;

        assert(mObservations_first.size() == mObservations_second.size());
        for(size_t j = 0; j < mObservations_first.size(); j++){
            //            cout << "j = " << j << endl;
            //            cout << (unsigned long)(mObservations_first[j]) << " " << mObservations_second[j] << endl;
            //            vpMPs[i]->mObservations[id2pKFmap[mObservations_first[j]]] = mObservations_second[j];
            //            if (mObservations_first[j] >= 0) {
            //            cout << "before" << " ";
            auto iter = map_ID_2_KF.find((unsigned long)(mObservations_first[j]));
            if (iter != map_ID_2_KF.end() && iter->second != NULL) {
                //                cout << "after" << " ";
                //                cout << iter->first << "; ";
                //                cout << iter->second->mnId << endl;
                //                cout << vpMPs[i]->nObs << " -> ";
                vpMPs[i]->AddObservation(iter->second, mObservations_second[j]);
                //                cout << vpMPs[i]->nObs << endl;
            }
            //            cout << endl;
            //            }
        }
        //        cout << endl;

        //loading mpRefKF
        unsigned long mRefKF = static_cast<unsigned long>(mRefKFVector[i]);
        //        vpMPs[i]->mpRefKF = id2pKFmap[mRefKF];
        //        if (mRefKF >= 0)
        //            vpMPs[i]->SetReferenceKeyFrame(map_ID_2_KF[(unsigned long)(mRefKF)]);
        //        else
        //            cout << "this should not happen!" << endl;
        assert(mRefKF >= 0);
        auto it = map_ID_2_KF.find(mRefKF);
        if (it != map_ID_2_KF.end()) {
            //            cout << mRefKF << "; " << endl;
            //            cout << it->second << "; " << endl;
            //            cout << it->second->mnId << endl;
            vpMPs[i]->SetReferenceKeyFrame(it->second);
        }
        else
            vpMPs[i]->SetBadFlag();
        //            vpMPs[i]->SetReferenceKeyFrame(NULL);
    }

    //    //Assigning Reference MapPoints..
    //    cout<<"Assigning Reference MapPoints.."<<endl;
    //    for(size_t i = 0; i < vpRMPs.size(); i++){
    //        //loading mObservations
    //        vector<int> mObservations_firstR = mObservations_firstVectorR[i];
    //        vector<int> mObservations_secondR = mObservations_secondVectorR[i];

    //        assert(mObservations_firstR.size() == mObservations_secondR.size());
    //        for(size_t j = 0; j < mObservations_secondR.size(); j++){
    //            //            vpRMPs[i]->mObservations[id2pKFmap[mObservations_first[j]]] = mObservations_second[j];
    //            //            if (mObservations_first[j] >= 0) {\auto iter = id2pKFmap.find((unsigned long)(mObservations_first[j]));
    //            auto iter = id2pKFmap.find((unsigned long)(mObservations_firstR[j]));
    //            if (iter != id2pKFmap.end() && iter->second != NULL) {
    //                vpRMPs[i]->AddObservation(iter->second, mObservations_secondR[j]);
    //            }
    //            //                auto it = id2pKFmap.find(mObservations_first[j]);
    //            //                if (it != id2pKFmap.end())
    //            //                    vpMPs[i]->AddObservation(it->second, mObservations_second[j]);
    //            //            }
    //        }
    //        //loading mpRefKF
    //        double mRefKF = mRefKFVectorR[i];
    //        //        vpRMPs[i]->mpRefKF = id2pKFmap[mRefKF];
    //        if (mRefKF >= 0)
    //            vpRMPs[i]->SetReferenceKeyFrame(id2pKFmap[mRefKF]);
    //    }

    vector<ORB_SLAM2::MapPoint*> vpRMPs;
    ifstream fin;
    cout << endl << "Loading Reference MapPoints mnIds from RMp_idList.txt" << endl;
    string strFile = map_path + "/RMp_idList.txt";

    fin.open(strFile.c_str());
    if (fin.is_open()) {
        unsigned long mpId;
        while (fin >> mpId) {
            //            cout << mpId << " ";
            auto it = map_ID_2_Pt.find(mpId);
            if (it != map_ID_2_Pt.end())
                vpRMPs.push_back(it->second);
        }
    }
    //    cout << endl;
    fin.close();

    //load KerFrames, MapPoints, Reference MapPoints, Database
    cout<<"AddKeyFrame.." << endl;
    for(size_t i = 0; i < vpKFs.size(); i++)
        if (!vpKFs[i]->isBad())
            mpMap->AddKeyFrame(vpKFs[i]);

    cout<<"AddMapPoint.." << endl;
    for(size_t i = 0; i < vpMPs.size(); i++)
        if (!vpMPs[i]->isBad())
            mpMap->AddMapPoint(vpMPs[i]);

    cout<<"SetReferenceMapPoints.." << endl;
    mpMap->SetReferenceMapPoints(vpRMPs);

    cout<<"add mpKeyFrameDatabase.." << endl;
    for(size_t i = 0; i < vpKFs.size(); i++)
        if (!vpKFs[i]->isBad())
            mpKeyFrameDatabase->add(vpKFs[i]);

    // set the 1st KF as reloc reference
    // unsigned long firstKFId;
    size_t i = 0;
    for(; i < vpKFs.size(); i++)
        if (!vpKFs[i]->isBad()) {
            //         firstKFId = vpKFs[i]->mnId;
            break ;
        }
    //    long unsigned int lastKFId = vpKFs[vpKFs.size()-1]->mnId;
    //    ORB_SLAM2::Frame Frame(lastKFId);

    cout << "max kf id = " << max_KF_Id << endl;
    cout << "reloc kf id = " << vpKFs[i]->mnId << endl;
    cout << "reloc kf pointer = " << vpKFs[i] << endl;

    // mpTracker->SetLastKeyFrameId(firstKFId);
    //    mpTracker->SetLastKeyFrame(vpKFs[vpKFs.size()-1]);
    mpTracker->SetLastKeyFrame(vpKFs[i]);
    mpTracker->SetReferenceKeyFrame(vpKFs[i]);

    //
    KeyFrame::nNextId = max_KF_Id + 1;
    Frame::nNextId = max_KF_Id + 1;
    Frame::mbInitialComputations = true;
    MapPoint::nNextId = max_Pt_Id + 1;

}

void System::SaveMap(const std::string & map_path) {
    //
    //    if (!boost::filesystem::create_directories(map_path)) {
    //        return ;
    //    }
    boost::filesystem::create_directories(map_path);
    std::cout << "Successfully Created Dir For Maps!" << std::endl;

    // create sub folders for each map component
    boost::filesystem::path dir_KF(map_path + "/KeyFrames");
    if(!(boost::filesystem::exists(dir_KF))){
        if (boost::filesystem::create_directory(dir_KF))
            std::cout << "Successfully Created Sub-Dir For Key Frames!" << std::endl;
    }

    boost::filesystem::path dir_Map(map_path + "/MapPoints");
    if(!(boost::filesystem::exists(dir_Map))){
        if (boost::filesystem::create_directory(dir_Map))
            std::cout << "Successfully Created Sub-Dir For Map Points!" << std::endl;
    }

    //    boost::filesystem::path dir_RMap(map_path + "/RMapPoints");
    //    if(!(boost::filesystem::exists(dir_RMap))){
    //        if (boost::filesystem::create_directory(dir_RMap))
    //            std::cout << "Successfully Created Dire For Ref Map Points!" << std::endl;
    //    }

    //
    // grab all keyframes
    vector<ORB_SLAM2::KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),ORB_SLAM2::KeyFrame::idComp);

    cout << endl << "Saving KF mnIds to KF_idList.txt" << endl;
    string strFile = map_path + "/KF_idList.txt";

    ofstream fout(strFile.c_str());
    if (!fout) {
        cout << "func SaveMap: ERROR at creating " << strFile << endl;
        return ;
    }
    fout << fixed;
    fout << "keyframe mnId = "<<endl;
    //save keyframes///////////////////////////////////////////////////////////////////////
    cout << endl << "Saving KeyFrames.." << endl;
    cout<<"KeyFrame size = "<<vpKFs.size()<<endl;
    int count = 0;
    for(size_t i=0; i<vpKFs.size(); i++)//vpKFs.size()
    {
        //        cout << "exporting keyframe " << i << endl;

        ORB_SLAM2::KeyFrame* pKF = vpKFs[i];

        //if(pKF->isBad())//might cause missing KF in covisibility graph
        //    continue;
        fout << pKF->mnId << " ";
        //        string KFfileName = ros::package::getPath("ORB_SLAM")+"/bin/KeyFrames/"+"keyframe_"+boost::to_string(count++) + ".yml";
        char str_index[20];
        sprintf(str_index, "%06d", count);
        count ++;
        //        sprintf(str_index, "%032d", pKF->mnId);
        string KFfileName = map_path + "/KeyFrames/keyframe_" + std::string(str_index) + ".yml";

        cv::FileStorage fs(KFfileName.c_str(), cv::FileStorage::WRITE);

        pKF->ExportToYML(fs);

        fs.release();
    }

    fout << endl << "databaseId = " << endl;
    for (std::set<ORB_SLAM2::KeyFrame*>::iterator it=mpKeyFrameDatabase->mvpKFset.begin(); it != mpKeyFrameDatabase->mvpKFset.end(); ++it){
        fout << " " << (*it)->mnId;
    }
    fout.close();

    //save mappoints////////////////////////////////////////////////////////////////////////////////////
    vector<ORB_SLAM2::MapPoint*> vpMPs = mpMap->GetAllMapPoints();
    sort(vpMPs.begin(),vpMPs.end(),ORB_SLAM2::MapPoint::isLessID);
    cout << endl << "Saving MapPoints.." << endl;

    count = 0;
    int Nmap = vpMPs.size();
    cout<<"MapPoint size = "<<Nmap<<endl;
    for(size_t i=0; i<Nmap; i++)//vpMPs.size()
    {
        ORB_SLAM2::MapPoint* pMP = vpMPs[i];

        //if(pMP->isBad())
        //    continue;

        //cout<<"mappoint mnId = "<<pMP->mnId<<endl;
        //        string MPfileName = ros::package::getPath("ORB_SLAM")+"/bin/MapPoints/"+"mappoint_"+boost::to_string(count++) + ".yml";
        char str_index[20];
        sprintf(str_index, "%08d", count);
        count ++;
        //        sprintf(str_index, "%032d", pMP->mnId);
        string MPfileName = map_path + "/MapPoints/mappoint_" + std::string(str_index) + ".yml";

        cv::FileStorage fs(MPfileName.c_str(), cv::FileStorage::WRITE);

        pMP->ExportToYML(fs);

        fs.release();
    }

    //save reference mappoints////////////////////////////////////////////////////////////////////////////////////
    vector<ORB_SLAM2::MapPoint *> vpRMPs = mpMap->GetReferenceMapPoints();
    sort(vpRMPs.begin(), vpRMPs.end(), ORB_SLAM2::MapPoint::isLessID);
    cout << endl
         << "Saving Reference MapPoints mnIds to RMp_idList.txt" << endl;
    strFile = map_path + "/RMp_idList.txt";

    fout.open(strFile.c_str());
    if (!fout) {
        cout << "func SaveMap: ERROR at creating " << strFile << endl;
        return ;
    }
    fout << fixed;
    //    f << "ref map point mnId = "<<endl;
    for(size_t i=0; i<vpRMPs.size(); i++)
        fout<<vpRMPs[i]->mnId<<" ";
    fout<<endl;
    fout.close();
    
    // Export map point to PCD file; for viz
    /* cout << endl << "Saving MapPoints to PCD file" << endl;
    strFile = map_path + "/Map_3D_cloud.pcd";
    fout.open(strFile.c_str());
    fout << "# .PCD v.7 - Point Cloud Data file format" << endl;
    fout << "VERSION .7" << endl;
    fout << "FIELDS x y z rgb" << endl;
    fout << "SIZE 4 4 4 4" << endl;
    fout << "TYPE F F F F" << endl;
    fout << "COUNT 1 1 1 1" << endl;
    fout << "WIDTH " << Nmap << endl;
    fout << "HEIGHT 1" << endl;
    fout << "VIEWPOINT 0 0 0 1 0 0 0" << endl;
    fout << "POINTS " << Nmap << endl;
    fout << "DATA ascii" << endl;
    //
    fout << fixed;
    for (size_t i=0; i<Nmap; ++i) {
      ORB_SLAM2::MapPoint* pMP = vpMPs[i];
      cv::Mat p3d = pMP->GetWorldPos();
      for (size_t j=0; j<3; ++j)
    fout << p3d[j] << " ";
      fout << pMP->GetFoundRatio() << endl;
    }
    fout.close();
*/

}

#endif

} // namespace ORB_SLAM2
