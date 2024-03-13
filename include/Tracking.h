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

#ifndef TRACKING_H
#define TRACKING_H

// #include <opencv2/core/core.hpp>
// #include <opencv2/features2d/features2d.hpp>

#include "Viewer.h"
#include "FrameDrawer.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
#include "ORBextractor.h"
#include "Initializer.h"
#include "MapDrawer.h"
#include "System.h"

#include "Hashing.h"
#include "ChArUco.h"
#include "Observability.h"
#include "MacroDefinitions.h"

#include <set>
#include <utility>
#include <algorithm>
#include "nav_msgs/Odometry.h"

#include <Eigen/Dense>
using namespace Eigen;

#include <opencv2/core/eigen.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgcodecs/legacy/constants_c.h>
#include <opencv2/imgproc/types_c.h>
// #include <opencv2/xphoto.hpp>

#include <mutex>


/* --- options of initialization --- */
// #define INIT_WITH_ARUCHO


/* --- options for ground truth generation, by running at a slow rate and additional optimization iterations --- */
// #define GROUND_TRUTH_GEN_MODE


/* --- options for outdoor challenging sequences such as RobotCar and XWing --- */
// #define ENABLE_WHITE_BALANCE
// #define ENABLE_LARGE_SEARCH_WINDOW


/* --- options of enabling anticipation in motion-based tracking --- */
//
// NOTE
// For simulating closed-loop performance with open-loop benchmarks
// By enabling PRED_WITH_ODOM, the GT trajectory is loaded and purturbed 
// with random error.  It is then utilized as pose prediction (replacement 
// of constant motion model) in tracking thread.
//
// #define PRED_WITH_ODOM

/* --- options of anticipating poses with closed-loop planner  --- */
//
// NOTE
// For closed-loop navigation application ONLY
// By ENABLE_PLANNER_PREDICTION, please make sure the the trajectory state predictor 
// package is included in your catkin workspace:
// https://github.gatech.edu/ivabots/trajectory_state_predictor
// Otherwise, you might write your own predictor by grabbing output from the controller
//
// #define ENABLE_PLANNER_PREDICTION

#ifdef ENABLE_PLANNER_PREDICTION
  #include <trajectory_state_predictor/trajectory_state_predictor.h>
#endif

/* --- options of key-frame insert condition --- */
// #define SPARSE_KEYFRAME_COND


/* --- options of baseline methods --- */
// #define ORB_SLAM_BASELINE


/* --- options to priortize feature matching wrt local map --- */
#ifndef ORB_SLAM_BASELINE

    /* --- options of additional search after pose estimation --- */
    #define DELAYED_MAP_MATCHING

    /* --- options of turning on hashing-support in SLAM modules --- */
    // #define MAP_SIZE_TRIGGER_HASHING  2000 // 4000 //
    // #define ONLINE_TABLE_SELECTION
    // #define LOCAL_SEARCH_USING_HASHING
    // // #define ENABLE_TIME_CONTROL_FOR_TRACKING        // for tracking time control
    // #define RELOCALISATION_USING_HASHING

    /* --- options to priortize feature matching wrt local map --- */
    //#define RANDOM_FEATURE_MAP_MATCHING
    //#define LONGLIVE_FEATURE_MAP_MATCHING
    #define GOOD_FEATURE_MAP_MATCHING
    // include frame-by-frame matchings as prior term in good matching
    // #define FRAME_MATCHING_INFO_PRIOR
    // pre-compute Jacobian for next frame at the end of tracking
    // TODO disable it when using map hash; check the latency vs. performance
    #define PRECOMPUTE_WITH_MOTION_MODEL

    /* --- parameters used in good feature --- */
    #define USE_INFO_MATRIX
    //#define USE_HYBRID_MATRIX
    //#define USE_OBSERVABILITY_MATRIX

    // limit the budget of computing matrices of existing matches at current frame to 2ms
    #define MATRIX_BUDGET_REALTIME  0.002
    // limit the budget of predicting matrices at next frame to 2ms
    #define MATRIX_BUDGET_PREDICT   0.002

    // For low-power devices with 2-cores, disable multi-thread matrix building
    #define USE_MULTI_THREAD        true // false //

#endif

/* --- options to fair comparison wrt other VO pipelines --- */
//#define DISABLE_RELOC
// time to init tracking with full feature set
#define TIME_INIT_TRACKING          5 // 10 //
#define MAX_FRAME_LOSS_DURATION     999 // 5
//#define INITIALIZE_USING_BASELINE


/* --- options of non-necessary viz codes --- */
// when running on long-term large-scale dataset, this will save us a lot of time!
#define DISABLE_MAP_VIZ


/* --- options of debug --- */
//#define SIMU_MOTION_BLUR
//#define TIMECOST_VERBOSE
//#define LMKNUM_VERBOSE
#define REALTIME_TRAJ_LOGGING



namespace ORB_SLAM2
{

class Viewer;
class FrameDrawer;
class Map;
class LocalMapping;
class LoopClosing;
class System;


class Tracking
{  

public:
    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor);

    // for test only
    Tracking(const cv::Mat K, const cv::Mat DistCoef) {
        K.copyTo(mK);
        DistCoef.copyTo(mDistCoef);
    }


    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    cv::Mat GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp);
    cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp);
    cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);

    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopClosing(LoopClosing* pLoopClosing);
    void SetViewer(Viewer* pViewer);
    void SetHashHandler(HASHING::MultiIndexHashing* pHashHandler);

    // Load new settings
    // The focal lenght should be similar or scale prediction will fail when projecting points
    // TODO: Modify MapPoint::PredictScale to take into account focal lenght
    void ChangeCalibration(const string &strSettingPath);

    // Use this function if you have deactivated local mapping and you only want to localize the camera.
    void InformOnlyTracking(const bool &flag);


    ~Tracking();

    // init file stream for writing real time tracking result
    void SetRealTimeFileStream(string fNameRealTimeTrack);

    void updateORBExtractor();
    
    void BufferingOdom(const double & timeStamp,
                       const double & tx, const double & ty, const double & tz,
                       const double & qw, const double & qx, const double & qy, const double & qz);
    
    void ResetOdomBuffer();

    //    void BufferingOdom(const nav_msgs::Odometry::ConstPtr& msg);
    
    void PredictingOdom(const double & time_prev, const double & time_curr,
                        cv::Mat & T_se);

    inline void ForceReloc() {
        // cout << "start force reloc" << endl;
        mState = LOST;
        //        mvpLocalMapPoints = mpMap->GetAllMapPoints();
        //        if (mvpLocalKeyFrames.size() > 0)
        //            mpReferenceKF = mvpLocalKeyFrames[mvpLocalKeyFrames.size()-1];
        //        else
        //            mvpLocalKeyFrames = NULL;
        //        mCurrentFrame.mpReferenceKF = pKFcur;
        cout << "done with force reloc" << endl;
    }

    inline void ForceInit() {
        mState = NOT_INITIALIZED;
        cout << "done with force reloc" << endl;
    }

public:

    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3
    };

    enum eLocalMapSet{
        CovisOnly = 1,
        HashOnly  = 2,
        Combined  = 3
    };

    eTrackingState mState;
    eTrackingState mLastProcessedState;

    // Input sensor
    int mSensor;

    // Current Frame
    Frame mCurrentFrame;
    cv::Mat mImGray;

    arma::mat mCurrentInfoMat;
    int mnInitStereo;

    // Observability computation
    Observability * mObsHandler;

    size_t num_good_constr_predef;
    //    double ratio_good_inlier_predef;
    //    size_t num_good_feature_found;

    double time_track_budget;

    //    int mToMatchMeasurement;
    //    int mMatchedLocalMapPoint;

    //
    bool first_hit_tracking;
    double time_frame_init;
    double camera_fps;

#ifdef ENABLE_ANTICIPATION_IN_GRAPH
    double mVFrameInteval;
#endif

    //    vector<LmkSelectionInfo> obs_thres_arr;
    //    vector<FramePose> mFramePoseSeq;
    //    vector<std::pair<double, int> > mFrameInlierSeq;

    // Time log
    std::vector<TrackingLog> mFrameTimeLog;
    TrackingLog logCurrentFrame;

    //
    void BucketingMatches(const Frame *pFrame, vector<GoodPoint> & mpBucketed);
    void LongLivedMatches(const Frame *pFrame, vector<GoodPoint> & mpLongLived);
    void RanSACMatches(const Frame *pFrame, vector<GoodPoint> & mpRanSAC);

    // Initialization Variables (Monocular)
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    std::list<cv::Mat> mlRelativeFramePoses;
    std::list<KeyFrame *> mlpReferences;
    std::list<double> mlFrameTimes;
    std::list<bool> mlbLost;

    // True if local mapping is deactivated and we are performing only localization
    bool mbOnlyTracking;

#ifdef LOCAL_SEARCH_USING_HASHING
    
    inline void StoreLocalMapPointsByCoVis(const std::vector<MapPoint *> &LocalMapPoints)
    {
        mvpLocalMapPointsByCoVis.clear();
        mvpLocalMapPointsByCoVis = LocalMapPoints;
    }

    inline void RestoreLocalMapPoints(std::vector<MapPoint *> &LocalMapPoints)
    {
        //    LocalMapPoints = mvpLocalMapPointsBackup;
        mvpLocalMapPointsByCoVis.swap(LocalMapPoints);
    }

    inline bool BackupLocalMapPointsEmpty()
    {
        return (mvpLocalMapPointsByCoVis.size()==0);
    }
    
    void UpdateLocalPointsByHashing(eLocalMapSet eLocalMap);

    bool UpdateQueryNumByHashTable(const double time_limit);

#endif
    
    bool mbMapHashOTS;
    bool mbMapHashTriggered;
    
    void Reset();
    void appendMatrixToTextFile(const cv::Mat& matrix, const std::string& filename, double timestamp);

    inline void ResetInitNumFrame()
    {
        mFrameAfterInital = 0;
    }
    
    inline void SetReferenceKeyFrame(KeyFrame * pKF) {
        mpReferenceKF = pKF;
    }
    
    inline void SetLastKeyFrame(KeyFrame * pKF) {
        mpLastKeyFrame = pKF;
        mnLastKeyFrameId = pKF->mnId;
    }

    /*
    inline void SetLastKeyFrameId(unsigned long id) {
    mnLastKeyFrameId = id;
    }
*/

    //
    //    void ResetOdomPlanned()
    //    {
    //        mvOdomBuf.clear();
    //        mOdomLBAIdx = 0;
    //        mOdomTrackIdx = 0;
    //    }

    //    void AddOdomPlanned(const double & odom_stamp, const cv::Mat & odom_Tcw)
    //    {
    //        assert(odom_Twc.rols == 4 && odom_Twc.cols == 4);
    //        mvOdomPlanned.push_back({odom_stamp, odom_Tcw});
    //    }

protected:

    // Main tracking function. It is independent of the input sensor.
    void Track();

    // Map initialization for stereo and RGB-D
    void StereoInitialization();

    // Map initialization for monocular
    void MonocularInitialization();
    void CreateInitialMapMonocular();

    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrame();
    void UpdateLastFrame();
    bool TrackWithMotionModel();

    bool PredictRelMotionFromBuffer(const double & time_prev, const double & time_curr,
                                    cv::Mat & T_rel);

#ifdef ENABLE_PLANNER_PREDICTION
    inline bool PredictRelMotionFromCallback(const double & time_curr, const double & time_next,
                                             cv::Mat & T_rel) {
      if (!mpStatePred || time_next <= time_curr) {
	std::cout << "func PredictRelMotionFromCallback: invalid input param!" << std::endl;
	std::cout << setprecision(8) << "time_curr = " << time_curr << "; time_next = " << time_next  << std::endl;
        return false;
      }
      return mpStatePred->getRelativePose(time_curr, time_next-time_curr, T_rel);
    }
#endif

    bool Relocalization();

    void UpdateLocalMap();
    void UpdateLocalPoints();
    void UpdateLocalKeyFrames();

    bool TrackLocalMap();

    int SearchLocalPoints();
    bool SearchAdditionalMatchesInFrame(const double time_for_search, Frame & F);

    void PredictJacobianNextFrame(const double time_for_predict, const size_t pred_horizon);

    bool NeedNewKeyFrame();
    //
    bool NeedNewKeyFrame_Temp();
    bool NeedNewKeyFrame_Experimental();
    void CreateNewKeyFrame();


    void PlotFrameWithPointMatches();

    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    bool mbVO;

    //Other Thread Pointers
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopClosing;
    HASHING::MultiIndexHashing* mpHashMethod;

    //ORB
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    ORBextractor* mpIniORBextractor;

    //BoW
    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;

    // Initalization (only for monocular)
    Initializer* mpInitializer;

    //Local Map
    KeyFrame* mpReferenceKF;
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    std::vector<MapPoint*> mvpLocalMapPoints;
    std::vector<MapPoint*> mvpLocalMapPointsByCoVis;
    std::vector<MapPoint*> mvpLeftMapPointsByHashing;
    std::vector<MapPoint*> mvpMapPointsByHashing;
    std::vector<int> mvpQueriedFeatures;
    // System
    System* mpSystem;
    
    //Drawers
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    //Map
    Map* mpMap;

    // #ifdef ENABLE_WHITE_BALANCE

    // cv::Ptr<cv::xphoto::WhiteBalancer> mpBalancer;

    // #endif

    //Calibration matrix
    cv::Mat mK;
    cv::Mat mK_ori, mDistCoef, mR, mP;
    float mbf;
    //
    // for right camera undistortion
    cv::Mat mK_right, mDistCoef_right, mR_right, mP_right;

    //
    cv::Mat mMap1_l, mMap2_l, mMap1_r, mMap2_r;

    //New KeyFrame rules (according to fps)
    //    int mMinFrames;
    //    int mMaxFrames;
    unsigned long mMinFrames;
    unsigned long mMaxFrames;

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;

    // frame counter after initialization
    size_t mFrameAfterInital;
    
    size_t nFrameSinceLast;

    size_t mbTrackLossAlert;

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;

    //Current matches in frame
    int mnMatchesInliers;

    // Number of visible map points at current KF
    double mNumVisibleMpt;

    //Last Frame, KeyFrame and Relocalisation Info
    KeyFrame* mpLastKeyFrame;
    Frame mLastFrame;
    unsigned long mnLastKeyFrameId;
    unsigned long mnLastRelocFrameId;

    //Motion Model
    cv::Mat mVelocity;

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    std::list<MapPoint *> mlpTemporalPoints;

#ifdef ENABLE_PLANNER_PREDICTION
    TrajectoryStatePredictor * mpStatePred;
#endif
    
    // planned odom for anticipation in good graph
    // 1st: timeStamp;   2nd: mTcw
    //    std::vector<std::pair<double, cv::Mat>> mvOdomPlanned;
    int mOdomTrackIdx, mOdomLBAIdx;

    //
    //    int budget_matching_in_track = 150; // 60; // 100; //

    std::ofstream f_realTimeTrack;
    
    //
#ifdef INIT_WITH_ARUCHO 
    ChArUco * mpCharuco;
    cv::Mat mTw_align;
#endif
    
    std::mutex mMutexOdomBuf;
    std::vector<OdometryLog> mvOdomBuf;
    cv::Mat Tb2c, Tc2b;

};

} // namespace ORB_SLAM2

#endif // TRACKING_H
