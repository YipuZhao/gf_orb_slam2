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

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Frame.h"
#include "Hashing.h"
#include "MacroDefinitions.h"

#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

// debug option
//#define DEBUG_VERBOSE

// good graph switch macro
// #define ENABLE_GOOD_GRAPH
// reference methods to compare against good graph
// #define ENABLE_SLIDING_WINDOW_FILTER
// #define ENABLE_COVIS_GRAPH

// number of free KF in local BA to trigger good graph
// for EuRoC benchmark
#define GOOD_GRAPH_KF_THRES     30
#define GOOD_GRAPH_KF_MAXSZ     60
// // for FPV benchmark
// #define GOOD_GRAPH_KF_THRES     20
// #define GOOD_GRAPH_KF_MAXSZ     40

// TODO set threshold on fixed KF size (to reduce overhead in Jacobian)

//#define TRUE_MATCHING_RATIO     0.35 // 0.4 // 0.5

// min. number of local BA before accpeting a KF as fixed
#define GOOD_GRAPH_FIXED_THRES  1 // 0 // default //

#define GOOD_GRAPH_TIME_LOGGING


namespace ORB_SLAM2
{

class LoopClosing;

class Optimizer
{
public:
    void static BundleAdjustment(const std::vector<KeyFrame*> &vpKF, const std::vector<MapPoint*> &vpMP,
                                 int nIterations = 5, bool *pbStopFlag=NULL, const unsigned long nLoopKF=0,
                                 const bool bRobust = true);
    void static GlobalBundleAdjustemnt(Map* pMap, int nIterations=5, bool *pbStopFlag=NULL,
                                       const unsigned long nLoopKF=0, const bool bRobust = true);

#ifdef ENABLE_GOOD_GRAPH
    void static LocalBundleAdjustment(KeyFrame *pKF, bool *pbStopFlag, Map *pMap,
                                      MappingLog & time_log, BudgetPredictParam * param_ = NULL);
#else
    void static LocalBundleAdjustment(KeyFrame *pKF, bool *pbStopFlag, Map *pMap,
                                      MappingLog & time_log);
#endif
    //
    void static LocalBundleAdjustment(KeyFrame *pKF, bool *pbStopFlag, Map *pMap,
                                      vector<size_t> &mvKeyFrameList, vector<size_t> &mvFixedFrameList,
                                      size_t &num_Point);

    //
    int static PoseOptimization(Frame* pFrame);
    int static PoseOptimization_Selected(Frame *pFrame, const vector<GoodPoint> & mpSorted);

    // if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise (mono)
    void static OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFrame *, set<KeyFrame *> > &LoopConnections,
                                       const bool &bFixScale);

    // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono)
    static int OptimizeSim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches1,
                            g2o::Sim3 &g2oS12, const float th2, const bool bFixScale);

    //
    static void convertKF2TVertex(KeyFrame *pKF, Eigen::Matrix<double, 12, 1> & pose_cam);
    static void convertKF2TVertex(KeyFrame *pKF, Eigen::Matrix<double, 11, 1> & pose_cam);
    static void convertSKF2TVertex(KeyFrame *pKF, Eigen::Matrix<double, 12, 1> & pose_cam);
    static void convertMP2TVertex(MapPoint *pMP, Eigen::Vector3d & pose_lmk);
    //
    static int estimateKFNum(const double & coe_a, const double & coe_b,
                             const double & coe_c, const double & coe_d,
                             const double & target_timecost);

    //#if defined ENABLE_GOOD_GRAPH && defined GOOD_GRAPH_TIME_LOGGING
    //    static double mTimeInsertVertex;
    //    static double mTimeJacobain;
    //    static double mTimeQuery;
    //    static double mTimeSchur;
    //    static double mTimePerm;
    //    static double mTimeCholesky;
    //    static double mTimePost;
    //    static double mTimeOptim;
    //    //
    //    static void setTimerZero();
    //#endif

    //    void AssembleVirtualKF(std::vector<KeyFrame *> & vKF_pred);
    //    vector<cv::Mat> nVirtualOdom;
    //    double static camera_fps;
    //    double static coe_a, coe_b, coe_c, coe_d;
    // SplineFunction static budget_predictor;
};

} //namespace ORB_SLAM

#endif // OPTIMIZER_H
