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

#include "LocalMapping.h"
#include "LoopClosing.h"
#include "ORBmatcher.h"
#include "Optimizer.h"

#ifdef ENABLE_ANTICIPATION_IN_BUDGET
#include <Eigen/LU>
#endif

#include <mutex>

namespace ORB_SLAM2
{

LocalMapping::LocalMapping(Map *pMap, const float bMonocular):
    mbMonocular(bMonocular), mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
    mbAbortBA(false), mbStopped(false), mbStopRequested(false), mbNotStop(false), mbAcceptKeyFrames(true)
{
#ifdef ENABLE_GOOD_GRAPH
    std::cout << "LocalMapping: good graph enabled. " << std::endl;
#elif defined ENABLE_SLIDING_WINDOW_FILTER
    std::cout << "LocalMapping: sliding window filter enabled! " << std::endl;
#elif defined ENABLE_COVIS_GRAPH
    std::cout << "LocalMapping: covisible graph enabled! " << std::endl;
#else
    std::cout << "LocalMapping: default ORB2 in local BA enabled! " << std::endl;
#endif
#ifdef ENABLE_ANTICIPATION_IN_BUDGET
    //    mKFNumMat.Zero();
    //    mBudgetMat.Zero();
    mRunningRow = 0;
    mbFullyLoaded = false;
    //
    // prior coeffients for EuRoC sequences
    mParam.coe_a = -4.626365363955979e-04; // -2.762403356776874e-04;
    mParam.coe_b = 0.122198203669508; // 0.109476983548765;
    mParam.coe_c = -0.203900111858952; // -0.162915402955300;
    mParam.coe_d = 75.720220172857170; // 71.289612325498380;

    //
    // prior coeffients for EuRoC sequences on Jetson TX2
    // mParam.coe_a = 0.00178178408800715;
    // mParam.coe_b = -0.151759389787934;
    // mParam.coe_c = 23.5952776017384;
    // mParam.coe_d = 14.8044507431834;

    //
    // prior coeffients for FPV sequences
    //    mParam.coe_a = -0.000615979109315896; // -0.000580387577168096;
    //    mParam.coe_b = 0.105468647768499; // 0.103858878586380;
    //    mParam.coe_c = 0.351263613720075; // 0.488606421597128;
    //    mParam.coe_d = 40.6725054076365; // 38.1498525945427;


    //
    // prior coeffients for Gazebo closed-loop navigation (on PC)
    // mParam.coe_a = -5.989414432992870e-04;
    // mParam.coe_b = 0.031058229077929;
    // mParam.coe_c = 8.965307762782414;
    // mParam.coe_d = 37.807563677029975;

    //
    // prior coeffients for Gazebo closed-loop navigation (on low-power)
    //TODO
    //TODO
    //TODO

    //
    mParam.match_ratio_log.clear();
    mParam.match_ratio_idx = 0;
    std::cout << "LocalMapping: graph anticipation enabled! " << std::endl;
#else
    std::cout << "LocalMapping: graph anticipation disabled! " << std::endl;
#endif
    //
    mvBATimeLog.clear();
}

void LocalMapping::SetLoopCloser(LoopClosing* pLoopCloser)
{
    mpLoopCloser = pLoopCloser;
}

void LocalMapping::SetHashHandler(HASHING::MultiIndexHashing *pHashHandler)
{
    mpHashMethod = pHashHandler;
}

void LocalMapping::SetTracker(Tracking *pTracker)
{
    mpTracker = pTracker;

#ifdef ENABLE_ANTICIPATION_IN_BUDGET
    // set budget in seconds
    // mParam.budget_per_frame = mpTracker->time_track_budget;
    mParam.budget_per_frame = 1.0f / mpTracker->camera_fps;
    mParam.avg_match_num    = double(mpTracker->num_good_constr_predef) / 2.0;
#endif
}

#ifdef LOGGING_KF_LIST
void LocalMapping::SetRealTimeFileStream(string fNameRealTimeBA)
{
    f_realTimeBA.open(fNameRealTimeBA.c_str());
    f_realTimeBA << fixed;
    f_realTimeBA << "#TimeStamp NumKF [IDs] NumFixF [IDs]" << std::endl;
}
#endif

void LocalMapping::Run()
{
    //
    arma::wall_clock timer;
    mbFinished = false;

    while(1)
    {
        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(false);

        // Check if there are keyframes in the queue
        bool bNeedNewKeyFrames = CheckNewKeyFrames();
        if(bNeedNewKeyFrames)
        {

#ifdef LOCAL_BA_TIME_LOGGING
            logCurrentKeyFrame.setNaN();
            logCurrentKeyFrame.frame_time_stamp = mlNewKeyFrames.front()->mTimeStamp;
#endif
            // BoW conversion and insertion in Map
            ProcessNewKeyFrame();

            // Check recent MapPoints
            MapPointCulling();

            // Triangulate new MapPoints
            CreateNewMapPoints();

            if (!CheckNewKeyFrames())
            {
                // Find more matches in neighbor keyframes and fuse point duplications
                SearchInNeighbors();
            }

            mbAbortBA = false;

            if (!CheckNewKeyFrames() && !stopRequested())
            {

                if (mpMap->KeyFramesInMap() > 2)
                {
#ifdef LOCAL_BA_TIME_LOGGING
                    // logCurrentKeyFrame.time_srh_more_neighbor = timer.toc();
                    timer.tic();
#endif

#ifdef LOGGING_KF_LIST
                    // Local BA & export the logged kf list
                    Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame, &mbAbortBA, mpMap,
                                                     mvKeyFrameList, mvFixedFrameList,
                                                     logCurrentKeyFrame.num_Point);
                    //
                    logCurrentKeyFrame.num_fixed_KF = mvFixedFrameList.size();
                    logCurrentKeyFrame.num_free_KF = mvKeyFrameList.size();
                    //
                    f_realTimeBA << setprecision(6) << mpCurrentKeyFrame->mTimeStamp << endl;
                    f_realTimeBA << setprecision(7) << mvKeyFrameList.size();
                    for (int i = 0; i < mvKeyFrameList.size(); ++i)
                        f_realTimeBA << " " << mvKeyFrameList[i];
                    f_realTimeBA << endl;
                    //
                    f_realTimeBA << mvFixedFrameList.size();
                    for (int i = 0; i < mvFixedFrameList.size(); ++i)
                        f_realTimeBA << " " << mvFixedFrameList[i];
                    f_realTimeBA << endl;
#else
                    // Local BA
#ifdef ENABLE_ANTICIPATION_IN_BUDGET
                    Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame, &mbAbortBA, mpMap,
                                                     logCurrentKeyFrame.num_fixed_KF, logCurrentKeyFrame.num_free_KF, logCurrentKeyFrame.num_Point,
                                                     logCurrentKeyFrame, &mParam);
#else
                    Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame, &mbAbortBA, mpMap,
                                                     logCurrentKeyFrame.num_fixed_KF, logCurrentKeyFrame.num_free_KF, logCurrentKeyFrame.num_Point,
                                                     logCurrentKeyFrame);
#endif

#endif

#ifdef LOCAL_BA_TIME_LOGGING
                    logCurrentKeyFrame.time_local_BA = timer.toc();
                    //#if defined ENABLE_GOOD_GRAPH && defined GOOD_GRAPH_TIME_LOGGING
                    //                    //
                    //                    logCurrentKeyFrame.time_gg_insert_vertex    = Optimizer::mTimeInsertVertex;
                    //                    logCurrentKeyFrame.time_gg_jacobian         = Optimizer::mTimeJacobain;
                    //                    logCurrentKeyFrame.time_gg_rnd_query        = Optimizer::mTimeQuery;
                    //                    logCurrentKeyFrame.time_gg_schur            = Optimizer::mTimeSchur;
                    //                    logCurrentKeyFrame.time_gg_permute          = Optimizer::mTimePerm;
                    //                    logCurrentKeyFrame.time_gg_cholesky         = Optimizer::mTimeCholesky;
                    //                    logCurrentKeyFrame.time_gg_postproc         = Optimizer::mTimePost;
                    //                    logCurrentKeyFrame.time_gg_optimization     = Optimizer::mTimeOptim;
                    //#endif
#endif
                }

                // Check redundant local Keyframes
                KeyFrameCulling();
            }

            mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);

#ifdef LOCAL_BA_TIME_LOGGING
            mvBATimeLog.push_back(logCurrentKeyFrame);
#endif

#ifdef LOCAL_SEARCH_USING_HASHING
            std::vector<MapPoint*> vpBuckUpMapPoints;
            mpTracker->RestoreLocalMapPoints(vpBuckUpMapPoints);
            UpdateHashTables(vpBuckUpMapPoints);
#endif

        }
        else if (Stop())
        {
            // Safe area to stop
            while (isStopped() && !CheckFinish())
            {
                usleep(3000);
            }
            if (CheckFinish())
                break;
        }

        ResetIfRequested();

        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(true);

        if (CheckFinish())
            break;

        usleep(3000);
    }

    SetFinish();
}

void LocalMapping::InsertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexNewKFs);
    mlNewKeyFrames.push_back(pKF);
    mbAbortBA=true;
}


bool LocalMapping::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexNewKFs);
    return(!mlNewKeyFrames.empty());
}

void LocalMapping::ProcessNewKeyFrame()
{
    {
        unique_lock<mutex> lock(mMutexNewKFs);
        mpCurrentKeyFrame = mlNewKeyFrames.front();
        mlNewKeyFrames.pop_front();
    }

#ifdef LOCAL_BA_TIME_LOGGING
    timer.tic();
#endif

    // Compute Bags of Words structures
    mpCurrentKeyFrame->ComputeBoW();

    // Associate MapPoints to the new keyframe and update normal and descriptor
    const vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();

    for(size_t i=0; i<vpMapPointMatches.size(); i++)
    {
        MapPoint* pMP = vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                if(!pMP->IsInKeyFrame(mpCurrentKeyFrame))
                {
                    //                    cout << "pMP.id: " << pMP->mnId << endl;
                    //                    cout << "before add obs: " << pMP->GetReferenceKeyFrame() << endl;
                    pMP->AddObservation(mpCurrentKeyFrame, i);
                    //                    cout << "after add obs: " << pMP->GetReferenceKeyFrame() << endl;
                    pMP->UpdateNormalAndDepth();
                    pMP->ComputeDistinctiveDescriptors();
                }
                else // this can only happen for new stereo points inserted by the Tracking
                {
                    mlpRecentAddedMapPoints.push_back(pMP);
                }
            }
        }
    }

    // Update links in the Covisibility Graph
    mpCurrentKeyFrame->UpdateConnections();

    // Insert Keyframe in Map
    mpMap->AddKeyFrame(mpCurrentKeyFrame);

#ifdef LOCAL_BA_TIME_LOGGING
    logCurrentKeyFrame.time_proc_new_keyframe = timer.toc();
#endif
}

void LocalMapping::MapPointCulling()
{
#ifdef LOCAL_BA_TIME_LOGGING
    timer.tic();
#endif

    // Check Recent Added MapPoints
    list<MapPoint*>::iterator lit = mlpRecentAddedMapPoints.begin();
    const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;

    int nThObs;
    if(mbMonocular)
        nThObs = 2;
    else
        nThObs = 3;
    const int cnThObs = nThObs;

    while(lit!=mlpRecentAddedMapPoints.end())
    {
        MapPoint* pMP = *lit;
        if(pMP->isBad())
        {
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(pMP->GetFoundRatio()<0.25f )
        {
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=2 && pMP->Observations()<=cnThObs)
        {
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=3)
            lit = mlpRecentAddedMapPoints.erase(lit);
        else
            lit++;
    }

#ifdef LOCAL_BA_TIME_LOGGING
    logCurrentKeyFrame.time_culling = timer.toc();
#endif
}

void LocalMapping::CreateNewMapPoints()
{
#ifdef LOCAL_BA_TIME_LOGGING
    timer.tic();
#endif

    // Retrieve neighbor keyframes in covisibility graph
    int nn = 10;
    if(mbMonocular)
        nn=20;
    const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

    ORBmatcher matcher(0.6,false);

    Eigen::MatrixXf Rcw1 = mpCurrentKeyFrame->GetRotation();
    Eigen::MatrixXf Rwc1 = Rcw1.transpose();
    Eigen::VectorXf tcw1 = mpCurrentKeyFrame->GetTranslation();
    Eigen::MatrixXf Tcw1;
    Tcw1.block<3,3>(0,0) = Rcw1;
    Tcw1.block<3,1>(0,3) = tcw1;

    Eigen::MatrixXf Ow1 = mpCurrentKeyFrame->GetCameraCenter();

    const float &fx1 = mpCurrentKeyFrame->fx;
    const float &fy1 = mpCurrentKeyFrame->fy;
    const float &cx1 = mpCurrentKeyFrame->cx;
    const float &cy1 = mpCurrentKeyFrame->cy;
    const float &invfx1 = mpCurrentKeyFrame->invfx;
    const float &invfy1 = mpCurrentKeyFrame->invfy;

    const float ratioFactor = 1.5f*mpCurrentKeyFrame->mfScaleFactor;

    int nnew=0;

    // Search matches with epipolar restriction and triangulate
    for(size_t i=0; i<vpNeighKFs.size(); i++)
    {
        if(i>0 && CheckNewKeyFrames())
            return;

        KeyFrame* pKF2 = vpNeighKFs[i];

        // Check first that baseline is not too short
        Eigen::MatrixXf Ow2 = pKF2->GetCameraCenter();
        Eigen::MatrixXf vBaseline = Ow2-Ow1;
        const float baseline = vBaseline.norm();

        if(!mbMonocular)
        {
            if(baseline<pKF2->mb)
                continue;
        }
        else
        {
            const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
            const float ratioBaselineDepth = baseline/medianDepthKF2;

            if(ratioBaselineDepth<0.01)
                continue;
        }

        // Compute Fundamental Matrix
        Eigen::MatrixXf F12 = ComputeF12(mpCurrentKeyFrame,pKF2);

        // Search matches that fullfil epipolar constraint
        vector<pair<size_t,size_t> > vMatchedIndices;
        matcher.SearchForTriangulation(mpCurrentKeyFrame,pKF2,F12,vMatchedIndices,false);

        Eigen::MatrixXf Rcw2 = pKF2->GetRotation();
        Eigen::MatrixXf Rwc2 = Rcw2.transpose();
        Eigen::VectorXf Tcw2 = pKF2->GetTranslation();
        Eigen::MatrixXf Tcw1;
        Tcw2.block<3,3>(0,0) = Rcw2;
        Tcw2.block<3,1>(0,3) = Rwc2;

        const float &fx2 = pKF2->fx;
        const float &fy2 = pKF2->fy;
        const float &cx2 = pKF2->cx;
        const float &cy2 = pKF2->cy;
        const float &invfx2 = pKF2->invfx;
        const float &invfy2 = pKF2->invfy;

        // Triangulate each match
        const int nmatches = vMatchedIndices.size();
        for(int ikp=0; ikp<nmatches; ikp++)
        {
            const int &idx1 = vMatchedIndices[ikp].first;
            const int &idx2 = vMatchedIndices[ikp].second;

            const cv::KeyPoint &kp1 = mpCurrentKeyFrame->mvKeysUn[idx1];
            const float kp1_ur=mpCurrentKeyFrame->mvuRight[idx1];
            bool bStereo1 = kp1_ur>=0;

            const cv::KeyPoint &kp2 = pKF2->mvKeysUn[idx2];
            const float kp2_ur = pKF2->mvuRight[idx2];
            bool bStereo2 = kp2_ur>=0;

            // Check parallax between rays
            Eigen::Vector3f  xn1;
            xn1 << (kp1.pt.x-cx1)*invfx1, (kp1.pt.y-cy1)*invfy1, 1.0;
            Eigen::Vector3f  xn2;
            xn2 << (kp2.pt.x-cx2)*invfx2, (kp2.pt.y-cy2)*invfy2, 1.0;

            Eigen::Vector3f  ray1 = Rwc1*xn1;
            Eigen::Vector3f  ray2 = Rwc2*xn2;
            const float cosParallaxRays = ray1.dot(ray2)/(ray1.norm())*(ray2.norm());

            float cosParallaxStereo = cosParallaxRays+1;
            float cosParallaxStereo1 = cosParallaxStereo;
            float cosParallaxStereo2 = cosParallaxStereo;

            if(bStereo1)
                cosParallaxStereo1 = cos(2*atan2(mpCurrentKeyFrame->mb/2,mpCurrentKeyFrame->mvDepth[idx1]));
            else if(bStereo2)
                cosParallaxStereo2 = cos(2*atan2(pKF2->mb/2,pKF2->mvDepth[idx2]));

            cosParallaxStereo = min(cosParallaxStereo1,cosParallaxStereo2);

            Eigen::VectorXf x3D;
            if(cosParallaxRays<cosParallaxStereo && cosParallaxRays>0 && (bStereo1 || bStereo2 || cosParallaxRays<0.9998))
            {
                // Linear Triangulation Method
                Eigen::MatrixXf A;
                A.row(0) = xn1(0)*Tcw1.row(2)-Tcw1.row(0);
                A.row(1) = xn1(1)*Tcw1.row(2)-Tcw1.row(1);
                A.row(2) = xn2(0)*Tcw2.row(2)-Tcw2.row(0);
                A.row(3) = xn2(1)*Tcw2.row(2)-Tcw2.row(1);

                //Compute Singular Value decomposition using Eigen.
                Eigen::MatrixXf w,u,vt;
                Eigen::JacobiSVD<Eigen::MatrixXf> svd( A, Eigen::ComputeFullV | Eigen::ComputeFullU );
                svd.computeU();
                svd.computeV();
                svd.solve(vt);

                x3D = vt.row(3).transpose();

                if(x3D(3)==0)
                    continue;

                // Euclidean coordinates
                x3D = x3D/x3D(3);

            }
            else if(bStereo1 && cosParallaxStereo1<cosParallaxStereo2)
            {
                x3D = mpCurrentKeyFrame->UnprojectStereo(idx1);
            }
            else if(bStereo2 && cosParallaxStereo2<cosParallaxStereo1)
            {
                x3D = pKF2->UnprojectStereo(idx2);
            }
            else
                continue; //No stereo and very low parallax

            Eigen::Vector3f x3Dt = x3D.transpose();

            //Check triangulation in front of cameras
            float z1 = Rcw1.row(2).dot(x3Dt)+tcw1(2);
            if(z1<=0)
                continue;

            float z2 = Rcw2.row(2).dot(x3Dt)+Tcw2(2);
            if(z2<=0)
                continue;

            //Check reprojection error in first keyframe
            const float &sigmaSquare1 = mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];
            const float x1 = Rcw1.row(0).dot(x3Dt)+tcw1(0);
            const float y1 = Rcw1.row(1).dot(x3Dt)+tcw1(1);
            const float invz1 = 1.0/z1;

            if(!bStereo1)
            {
                float u1 = fx1*x1*invz1+cx1;
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                if((errX1*errX1+errY1*errY1)>5.991*sigmaSquare1)
                    continue;
            }
            else
            {
                float u1 = fx1*x1*invz1+cx1;
                float u1_r = u1 - mpCurrentKeyFrame->mbf*invz1;
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                float errX1_r = u1_r - kp1_ur;
                if((errX1*errX1+errY1*errY1+errX1_r*errX1_r)>7.8*sigmaSquare1)
                    continue;
            }

            //Check reprojection error in second keyframe
            const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
            const float x2 = Rcw2.row(0).dot(x3Dt)+Tcw2(0);
            const float y2 = Rcw2.row(1).dot(x3Dt)+Tcw2(1);
            const float invz2 = 1.0/z2;
            if(!bStereo2)
            {
                float u2 = fx2*x2*invz2+cx2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                if((errX2*errX2+errY2*errY2)>5.991*sigmaSquare2)
                    continue;
            }
            else
            {
                float u2 = fx2*x2*invz2+cx2;
                float u2_r = u2 - mpCurrentKeyFrame->mbf*invz2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                float errX2_r = u2_r - kp2_ur;
                if((errX2*errX2+errY2*errY2+errX2_r*errX2_r)>7.8*sigmaSquare2)
                    continue;
            }

            //Check scale consistency
            Eigen::MatrixXf normal1 = x3D-Ow1;
            float dist1 = normal1.norm();

            Eigen::MatrixXf normal2 = x3D-Ow2;
            float dist2 = normal2.norm();

            if(dist1==0 || dist2==0)
                continue;

            const float ratioDist = dist2/dist1;
            const float ratioOctave = mpCurrentKeyFrame->mvScaleFactors[kp1.octave]/pKF2->mvScaleFactors[kp2.octave];

            /*if(fabs(ratioDist-ratioOctave)>ratioFactor)
                continue;*/
            if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
                continue;

            cv::Mat x3D_cv;
            cv::eigen2cv(x3D, x3D_cv);

            // Triangulation is succesfull
            MapPoint* pMP = new MapPoint(x3D_cv,mpCurrentKeyFrame,mpMap);

            pMP->AddObservation(mpCurrentKeyFrame,idx1);
            pMP->AddObservation(pKF2,idx2);

            mpCurrentKeyFrame->AddMapPoint(pMP,idx1);
            pKF2->AddMapPoint(pMP,idx2);

            pMP->ComputeDistinctiveDescriptors();

            pMP->UpdateNormalAndDepth();

            mpMap->AddMapPoint(pMP);
            mlpRecentAddedMapPoints.push_back(pMP);

#ifdef LOCAL_SEARCH_USING_HASHING
            arma::wall_clock timer11;
            timer11.tic();
            mpHashMethod->insert(pMP);
            mpTracker->logCurrentFrame.time_hash_insert += timer11.toc();
#endif

            nnew++;
        }
    }

#ifdef LOCAL_BA_TIME_LOGGING
    logCurrentKeyFrame.time_tri_new_map_point = timer.toc();
#endif
}

// TODO
// reduce the overhead of feature matching as part of good graph edge selection
void LocalMapping::SearchInNeighbors()
{
#ifdef LOCAL_BA_TIME_LOGGING
    timer.tic();
#endif

    // Retrieve neighbor keyframes
    int nn = 10;
    if(mbMonocular)
        nn=20;
    const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
    vector<KeyFrame*> vpTargetKFs;
    for(vector<KeyFrame*>::const_iterator vit=vpNeighKFs.begin(), vend=vpNeighKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        if(pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnId)
            continue;
        vpTargetKFs.push_back(pKFi);
        pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;

        // Extend to some second neighbors
        const vector<KeyFrame*> vpSecondNeighKFs = pKFi->GetBestCovisibilityKeyFrames(5);
        for(vector<KeyFrame*>::const_iterator vit2=vpSecondNeighKFs.begin(), vend2=vpSecondNeighKFs.end(); vit2!=vend2; vit2++)
        {
            KeyFrame* pKFi2 = *vit2;
            if(pKFi2->isBad() || pKFi2->mnFuseTargetForKF==mpCurrentKeyFrame->mnId || pKFi2->mnId==mpCurrentKeyFrame->mnId)
                continue;
            vpTargetKFs.push_back(pKFi2);
        }
    }


    // Search matches by projection from current KF in target KFs
    ORBmatcher matcher;
    vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(vector<KeyFrame*>::iterator vit=vpTargetKFs.begin(), vend=vpTargetKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;

        matcher.Fuse(pKFi,vpMapPointMatches);
    }

    // Search matches by projection from target KFs in current KF
    vector<MapPoint*> vpFuseCandidates;
    vpFuseCandidates.reserve(vpTargetKFs.size()*vpMapPointMatches.size());

    for(vector<KeyFrame*>::iterator vitKF=vpTargetKFs.begin(), vendKF=vpTargetKFs.end(); vitKF!=vendKF; vitKF++)
    {
        KeyFrame* pKFi = *vitKF;

        vector<MapPoint*> vpMapPointsKFi = pKFi->GetMapPointMatches();

        for(vector<MapPoint*>::iterator vitMP=vpMapPointsKFi.begin(), vendMP=vpMapPointsKFi.end(); vitMP!=vendMP; vitMP++)
        {
            MapPoint* pMP = *vitMP;
            if(!pMP)
                continue;
            if(pMP->isBad() || pMP->mnFuseCandidateForKF == mpCurrentKeyFrame->mnId)
                continue;
            pMP->mnFuseCandidateForKF = mpCurrentKeyFrame->mnId;
            vpFuseCandidates.push_back(pMP);
        }
    }

    matcher.Fuse(mpCurrentKeyFrame,vpFuseCandidates);


    // Update points
    vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(size_t i=0, iend=vpMapPointMatches.size(); i<iend; i++)
    {
        MapPoint* pMP=vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                pMP->ComputeDistinctiveDescriptors();
                pMP->UpdateNormalAndDepth();
            }
        }
    }

    // Update connections in covisibility graph
    mpCurrentKeyFrame->UpdateConnections();

#ifdef LOCAL_BA_TIME_LOGGING
    logCurrentKeyFrame.time_srh_more_neighbor = timer.toc();
#endif
}

Eigen::MatrixXf LocalMapping::ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2)
{
    Eigen::MatrixXf R1w = pKF1->GetRotation();
    Eigen::VectorXf t1w = pKF1->GetTranslation();
    Eigen::MatrixXf R2w = pKF2->GetRotation();
    Eigen::VectorXf t2w = pKF2->GetTranslation();

    Eigen::MatrixXf R12 = R1w*R2w.transpose();
    Eigen::MatrixXf t12 = -R1w*R2w.transpose()*t2w+t1w;

    Eigen::MatrixXf t12x = SkewSymmetricMatrix(t12);

    const cv::Mat &K1 = pKF1->mK;
    const cv::Mat &K2 = pKF2->mK;

    Eigen::MatrixXf K1_eigen;
    Eigen::MatrixXf K2_eigen;

    cv::cv2eigen(K1, K1_eigen);
    cv::cv2eigen(K2, K2_eigen);


    return (K1_eigen.transpose().inverse())*t12x*R12*(K2_eigen.inverse());
}

void LocalMapping::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopRequested = true;
    unique_lock<mutex> lock2(mMutexNewKFs);
    mbAbortBA = true;
}

bool LocalMapping::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(mbStopRequested && !mbNotStop)
    {
        mbStopped = true;
        cout << "Local Mapping STOP" << endl;
        return true;
    }

    return false;
}

bool LocalMapping::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool LocalMapping::stopRequested()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopRequested;
}

void LocalMapping::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);
    if(mbFinished)
        return;
    mbStopped = false;
    mbStopRequested = false;
    for(list<KeyFrame*>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++)
        delete *lit;
    mlNewKeyFrames.clear();

    cout << "Local Mapping RELEASE" << endl;
}

bool LocalMapping::AcceptKeyFrames()
{
    unique_lock<mutex> lock(mMutexAccept);
    return mbAcceptKeyFrames;
}

void LocalMapping::SetAcceptKeyFrames(bool flag)
{
    unique_lock<mutex> lock(mMutexAccept);
    mbAcceptKeyFrames=flag;
}

bool LocalMapping::SetNotStop(bool flag)
{
    unique_lock<mutex> lock(mMutexStop);

    if(flag && mbStopped)
        return false;

    mbNotStop = flag;

    return true;
}

void LocalMapping::InterruptBA()
{
    mbAbortBA = true;
}

void LocalMapping::KeyFrameCulling()
{
    // Check redundant keyframes (only local keyframes)
    // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
    // in at least other 3 keyframes (in the same or finer scale)
    // We only consider close stereo points
    vector<KeyFrame*> vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

    for(vector<KeyFrame*>::iterator vit=vpLocalKeyFrames.begin(), vend=vpLocalKeyFrames.end(); vit!=vend; vit++)
    {
        KeyFrame* pKF = *vit;
        if(pKF->mnId==0)
            continue;
        const vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();

        int nObs = 3;
        const int thObs=nObs;
        int nRedundantObservations=0;
        int nMPs=0;
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    if(!mbMonocular)
                    {
                        if(pKF->mvDepth[i]>pKF->mThDepth || pKF->mvDepth[i]<0)
                            continue;
                    }

                    nMPs++;
                    if(pMP->Observations()>thObs)
                    {
                        const int &scaleLevel = pKF->mvKeysUn[i].octave;
                        const map<KeyFrame*, size_t> observations = pMP->GetObservations();
                        int nObs=0;
                        for(map<KeyFrame*, size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                        {
                            KeyFrame* pKFi = mit->first;
                            if(pKFi==pKF)
                                continue;
                            const int &scaleLeveli = pKFi->mvKeysUn[mit->second].octave;

                            if(scaleLeveli<=scaleLevel+1)
                            {
                                nObs++;
                                if(nObs>=thObs)
                                    break;
                            }
                        }
                        if(nObs>=thObs)
                        {
                            nRedundantObservations++;
                        }
                    }
                }
            }
        }

        if(nRedundantObservations>0.9*nMPs)
            pKF->SetBadFlag();
    }
}

Eigen::MatrixXf LocalMapping::SkewSymmetricMatrix(const Eigen::MatrixXf &v)
{
  Eigen::Matrix<float,3,3> ret_matrix;
  ret_matrix <<   0, -v(2), v(1),
                  v(2), 0,-v(0),
                 -v(1),  v(0),0;
    return ret_matrix;
}

void LocalMapping::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    while(1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if(!mbResetRequested)
                break;
        }
        usleep(3000);
    }
}

void LocalMapping::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
        mlNewKeyFrames.clear();
        mlpRecentAddedMapPoints.clear();
        mbResetRequested=false;
    }
}

void LocalMapping::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool LocalMapping::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LocalMapping::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
    unique_lock<mutex> lock2(mMutexStop);
    mbStopped = true;
}

bool LocalMapping::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void LocalMapping::UpdateHashTables(std::vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> locker(mMutexUpdateHashTables);

    arma::wall_clock timer;
    timer.tic();
    for(auto it=vpMPs.begin(), itend=vpMPs.end(); it!=itend; it++){
        MapPoint* pMP = *it;
        if(!pMP || pMP->isBad())
            continue;

        mpHashMethod->insert(*it);
    }
    mpTracker->logCurrentFrame.time_hash_insert += timer.toc();
}

//void LocalMapping::UpdateHashTables(std::vector<MapPoint *> &vpMPs)
//{
//    arma::wall_clock timer;
//    timer.tic();
//    for(auto it=vpMPs.begin(), itend=vpMPs.end(); it!=itend; it++){
//        MapPoint* pMP = *it;
//        if(!pMP || pMP->isBad())
//            continue;

//        mpHashMethod->insert(*it);
//    }
//    mpTracker->logCurrentFrame.time_hash_insert = timer.toc();
//}

//void LocalMapping::UpdateHashTables(std::list<MapPoint *> &vpMPs)
//{
//    //    if(mpHashMethod->NeedUpdateHashTables()){
////    std::vector<MapPoint*> mvpCoviMPs;
////    if(!mpTracker->BackupLocalMapPointsEmpty()){
////        mpTracker->RestoreLocalMapPoints(mvpCoviMPs);
//        arma::wall_clock timer;
//        timer.tic();
//        for(auto it=vpMPs.begin(), itend=vpMPs.end(); it!=itend; it++){
//            MapPoint* pMP = *it;
//            if(!pMP || pMP->isBad())
//                continue;

//            mpHashMethod->insert(*it);
//        }
//        mpTracker->logCurrentFrame.time_hash_insert = timer.toc();
////    }
//    //        mpHashMethod->setUpdateHashTables(false);
//    //    }
//}

} // namespace ORB_SLAM2
