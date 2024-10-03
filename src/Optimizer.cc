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

#include "Optimizer.h"

#ifdef ENABLE_GOOD_GRAPH

#include <unordered_map>
#include "Thirdparty/SLAM++/include/good_graph_testbed/BAOptimizer.h" // BA types
// note that nothing else from SLAM++ needs to be included

int n_dummy_param = 0; // required by the DL solver, otherwise causes a link error

#endif

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

#include <Eigen/StdVector>

#include "Util.hpp"
#include "Converter.h"

#include <mutex>

namespace ORB_SLAM2
{

void Optimizer::GlobalBundleAdjustemnt(Map *pMap, int nIterations, bool *pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
{
    vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
    vector<MapPoint *> vpMP = pMap->GetAllMapPoints();
    BundleAdjustment(vpKFs, vpMP, nIterations, pbStopFlag, nLoopKF, bRobust);
}

void Optimizer::BundleAdjustment(const vector<KeyFrame *> &vpKFs, const vector<MapPoint *> &vpMP,
                                 int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
{
    vector<bool> vbNotIncludedMP;
    vbNotIncludedMP.resize(vpMP.size());

    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    long unsigned int maxKFid = 0;

    // Set KeyFrame vertices
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKF->GetPose()));
        vSE3->setId(pKF->mnId);
        vSE3->setFixed(pKF->mnId==0);
        optimizer.addVertex(vSE3);
        if(pKF->mnId>maxKFid)
            maxKFid=pKF->mnId;
    }

    const float thHuber2D = sqrt(5.99);
    const float thHuber3D = sqrt(7.815);

    // Set MapPoint vertices
    for(size_t i=0; i<vpMP.size(); i++)
    {
        MapPoint* pMP = vpMP[i];
        if(pMP->isBad())
            continue;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        const int id = pMP->mnId+maxKFid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        const map<KeyFrame*,size_t> observations = pMP->GetObservations();

        int nEdges = 0;
        //SET EDGES
        for(map<KeyFrame*,size_t>::const_iterator mit=observations.begin(); mit!=observations.end(); mit++)
        {

            KeyFrame* pKF = mit->first;
            if(pKF->isBad() || pKF->mnId>maxKFid)
                continue;

            nEdges++;

            const cv::KeyPoint &kpUn = pKF->mvKeysUn[mit->second];

            if(pKF->mvuRight[mit->second]<0)
            {
                Eigen::Matrix<double,2,1> obs;
                obs << kpUn.pt.x, kpUn.pt.y;

                g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
                e->setMeasurement(obs);
                const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                if(bRobust)
                {
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber2D);
                }

                e->fx = pKF->fx;
                e->fy = pKF->fy;
                e->cx = pKF->cx;
                e->cy = pKF->cy;

                optimizer.addEdge(e);
            }
            else
            {
                Eigen::Matrix<double,3,1> obs;
                const float kp_ur = pKF->mvuRight[mit->second];
                obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
                e->setMeasurement(obs);
                const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                e->setInformation(Info);

                if(bRobust)
                {
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber3D);
                }

                e->fx = pKF->fx;
                e->fy = pKF->fy;
                e->cx = pKF->cx;
                e->cy = pKF->cy;
                e->bf = pKF->mbf;

                optimizer.addEdge(e);
            }
        }

        if(nEdges==0)
        {
            optimizer.removeVertex(vPoint);
            vbNotIncludedMP[i]=true;
        }
        else
        {
            vbNotIncludedMP[i]=false;
        }
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(nIterations);

    // Recover optimized data

    //Keyframes
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        if(nLoopKF==0)
        {
            pKF->SetPose(Converter::toCvMat(SE3quat));
        }
        else
        {
            pKF->mTcwGBA.create(4,4,CV_32F);
            Converter::toCvMat(SE3quat).copyTo(pKF->mTcwGBA);
            pKF->mnBAGlobalForKF = nLoopKF;
        }
    }

    //Points
    for(size_t i=0; i<vpMP.size(); i++)
    {
        if(vbNotIncludedMP[i])
            continue;

        MapPoint* pMP = vpMP[i];

        if(pMP->isBad())
            continue;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));

        if(nLoopKF==0)
        {
            pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
            pMP->UpdateNormalAndDepth();
        }
        else
        {
            pMP->mPosGBA.create(3,1,CV_32F);
            Converter::toCvMat(vPoint->estimate()).copyTo(pMP->mPosGBA);
            pMP->mnBAGlobalForKF = nLoopKF;
        }
    }

}

int Optimizer::PoseOptimization(Frame *pFrame)
{
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    int nInitialCorrespondences=0;

    // Set Frame vertex
    g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
    vSE3->setId(0);
    vSE3->setFixed(false);
    optimizer.addVertex(vSE3);

    // Set MapPoint vertices
    const int N = pFrame->N;

    vector<g2o::EdgeSE3ProjectXYZOnlyPose*> vpEdgesMono;
    vector<size_t> vnIndexEdgeMono;
    vpEdgesMono.reserve(N);
    vnIndexEdgeMono.reserve(N);

    vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose*> vpEdgesStereo;
    vector<size_t> vnIndexEdgeStereo;
    vpEdgesStereo.reserve(N);
    vnIndexEdgeStereo.reserve(N);

    const float deltaMono = sqrt(5.991);
    const float deltaStereo = sqrt(7.815);


    {
        unique_lock<mutex> lock(MapPoint::mGlobalMutex);

        for(int i=0; i<N; i++)
        {
            MapPoint* pMP = pFrame->mvpMapPoints[i];
            if(pMP)
            {
                // Monocular observation
                if(pFrame->mvuRight[i]<0)
                {
                    nInitialCorrespondences++;
                    pFrame->mvbOutlier[i] = false;

                    Eigen::Matrix<double,2,1> obs;
                    const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                    obs << kpUn.pt.x, kpUn.pt.y;

                    g2o::EdgeSE3ProjectXYZOnlyPose* e = new g2o::EdgeSE3ProjectXYZOnlyPose();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
                    e->setMeasurement(obs);
                    const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(deltaMono);

                    e->fx = pFrame->fx;
                    e->fy = pFrame->fy;
                    e->cx = pFrame->cx;
                    e->cy = pFrame->cy;
                    cv::Mat Xw = pMP->GetWorldPos();
                    e->Xw[0] = Xw.at<float>(0);
                    e->Xw[1] = Xw.at<float>(1);
                    e->Xw[2] = Xw.at<float>(2);

                    optimizer.addEdge(e);

                    vpEdgesMono.push_back(e);
                    vnIndexEdgeMono.push_back(i);
                }
                else  // Stereo observation
                {
                    nInitialCorrespondences++;
                    pFrame->mvbOutlier[i] = false;

                    //SET EDGE
                    Eigen::Matrix<double,3,1> obs;
                    const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                    const float &kp_ur = pFrame->mvuRight[i];
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = new g2o::EdgeStereoSE3ProjectXYZOnlyPose();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
                    e->setMeasurement(obs);
                    const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                    e->setInformation(Info);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(deltaStereo);

                    e->fx = pFrame->fx;
                    e->fy = pFrame->fy;
                    e->cx = pFrame->cx;
                    e->cy = pFrame->cy;
                    e->bf = pFrame->mbf;
                    cv::Mat Xw = pMP->GetWorldPos();
                    e->Xw[0] = Xw.at<float>(0);
                    e->Xw[1] = Xw.at<float>(1);
                    e->Xw[2] = Xw.at<float>(2);

                    optimizer.addEdge(e);

                    vpEdgesStereo.push_back(e);
                    vnIndexEdgeStereo.push_back(i);
                }
            }

        }
    }

    if (nInitialCorrespondences < 3)
        return 0;


    // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
    // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
    const float chi2Mono[4]={5.991,5.991,5.991,5.991};
    const float chi2Stereo[4]={7.815,7.815,7.815, 7.815};
    const int its[4]={10,10,10,10};

    int nBad=0;
    for(size_t it=0; it<4; it++)
    {

        vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
        optimizer.initializeOptimization(0);
        optimizer.optimize(its[it]);

#ifdef GROUND_TRUTH_GEN_MODE
        optimizer.optimize(its[it] * 2);
#else
        optimizer.optimize(its[it]);
#endif

        nBad=0;
        for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
        {
            g2o::EdgeSE3ProjectXYZOnlyPose* e = vpEdgesMono[i];

            const size_t idx = vnIndexEdgeMono[i];

            if(pFrame->mvbOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if(chi2>chi2Mono[it])
            {
                pFrame->mvbOutlier[idx]=true;
                e->setLevel(1);
                nBad++;
            }
            else
            {
                pFrame->mvbOutlier[idx]=false;
                e->setLevel(0);
            }

            if(it==2)
                e->setRobustKernel(0);
        }

        for(size_t i=0, iend=vpEdgesStereo.size(); i<iend; i++)
        {
            g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = vpEdgesStereo[i];

            const size_t idx = vnIndexEdgeStereo[i];

            if(pFrame->mvbOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if(chi2>chi2Stereo[it])
            {
                pFrame->mvbOutlier[idx]=true;
                e->setLevel(1);
                nBad++;
            }
            else
            {
                e->setLevel(0);
                pFrame->mvbOutlier[idx]=false;
            }

            if(it==2)
                e->setRobustKernel(0);
        }

        if(optimizer.edges().size()<10)
            break;

        //
        // NOTE
        // the following condioning is applied to avoid pose optimization with empty graph
        // as suggested at https://github.com/raulmur/ORB_SLAM2/issues/211
        //
        //        if((nInitialCorrespondences-nBad)<5)
        //            break;
    }

    // Recover optimized pose and return number of inliers
    g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
    g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
    cv::Mat pose = Converter::toCvMat(SE3quat_recov);
    pFrame->SetPose(pose);

    return nInitialCorrespondences-nBad;
}



#ifdef ENABLE_GOOD_GRAPH
//
void Optimizer::convertKF2TVertex(KeyFrame *pKF, Eigen::Matrix<double, 12, 1> & pose_cam) {

    if (pKF == NULL)
        return ;

    // TODO figure out whether the inverse pose should be used
    g2o::SE3Quat gSE3 = Converter::toSE3Quat(pKF->GetPose());
    //    g2o::SE3Quat gSE3 = Converter::toSE3Quat(pKF->GetPoseInverse());

    // init the vertex
    Eigen::Quaternion<double> quat = gSE3.rotation();

    // no distortion
    Eigen::Vector3d c = gSE3.translation();
    pose_cam << c[0], c[1], c[2], quat.w(), quat.x(), quat.y(), quat.z(), pKF->fx, pKF->fy, pKF->cx, pKF->cy, 0;
}

void Optimizer::convertKF2TVertex(KeyFrame *pKF, Eigen::Matrix<double, 11, 1> & pose_cam) {

    if (pKF == NULL)
        return ;

    // TODO figure out whether the inverse pose should be used
    g2o::SE3Quat gSE3 = Converter::toSE3Quat(pKF->GetPose());
    //    g2o::SE3Quat gSE3 = Converter::toSE3Quat(pKF->GetPoseInverse());

    // init the vertex
    Eigen::Quaternion<double> quat = gSE3.rotation();
    /*
    quat.normalize();
    quat = quat.inverse();
    //Eigen::Matrix3d Q = quat.toRotationMatrix();
    //Q = Q.inverse().eval();
    //Q = Q.householderQr().householderQ();

    Eigen::Vector3d t_vec = gSE3.translation();
    //rotate
    Eigen::Vector3d c = quat * (-t_vec);
    //Eigen::Vector3d axis = C3DJacobians::v_RotMatrix_to_AxisAngle(Q);
    */
    Eigen::Vector3d axis;
    C3DJacobians::Quat_to_AxisAngle(quat, axis);

    // no distortion
    Eigen::Vector3d c = gSE3.translation();
    pose_cam << c[0], c[1], c[2], axis(0), axis(1), axis(2), pKF->fx, pKF->fy, pKF->cx, pKF->cy, 0;
    //    pose_cam << c[0], c[1], c[2], axis(0), axis(1), axis(2), pKF->fx, pKF->fy, 0, 0, 0;
}

void Optimizer::convertSKF2TVertex(KeyFrame *pKF, Eigen::Matrix<double, 12, 1> & pose_cam) {

    if (pKF == NULL)
        return ;

    g2o::SE3Quat gSE3 = Converter::toSE3Quat(pKF->GetPose());

    // init the vertex
    Eigen::Quaternion<double> quat = gSE3.rotation();
    /*
    quat.normalize();
    quat = quat.inverse();
    //Eigen::Matrix3d Q = quat.toRotationMatrix();
    //Q = Q.inverse().eval();
    //Q = Q.householderQr().householderQ();

    Eigen::Vector3d t_vec = gSE3.translation();
    //rotate
    Eigen::Vector3d c = quat * (-t_vec);
    //Eigen::Vector3d axis = C3DJacobians::v_RotMatrix_to_AxisAngle(Q);
    */
    Eigen::Vector3d axis;
    C3DJacobians::Quat_to_AxisAngle(quat, axis);

    // no distortion
    Eigen::Vector3d c = gSE3.translation();
    pose_cam << c[0], c[1], c[2], axis(0), axis(1), axis(2), pKF->fx, pKF->fy, pKF->cx, pKF->cy, 0, pKF->mb;
}

void Optimizer::convertMP2TVertex(MapPoint *pMP, Eigen::Vector3d & pose_lmk) {

    if (pMP == NULL)
        return ;

    pose_lmk = Converter::toVector3d(pMP->GetWorldPos());
}

int Optimizer::estimateKFNum(const double & coe_a, const double & coe_b,
                             const double & coe_c, const double & coe_d,
                             const double & target_timecost) {
    std::vector<double> t_sol;
    if (fabs(coe_a) < 1e-12) {
        if (fabs(coe_b) < 1e-12) {
            // solve the quadratic equation b*x + c = target_timecost
            return ceil((coe_d - target_timecost) / coe_c);
        }
        else {
            // solve the quadratic equation a*x^2 + b*x + c = target_timecost
            double coe_1_new = coe_c / coe_b;
            double coe_2_new = (coe_d - target_timecost) / coe_b;
            int flag = SolveQuadraticEquation(coe_1_new, coe_2_new, t_sol);
            if (flag == 2) {
                if (t_sol[0] < t_sol[1] && t_sol[0] > 0)
                    return ceil(t_sol[0]);
                else
                    return ceil(t_sol[1]);
            }
            else {
                return ceil(t_sol[0]);
            }
        }
    }
    else {
        // solve the cubic equation x^3 + a*x^2 + b*x + c = target_timecost
        double coe_1_new = coe_b / coe_a;
        double coe_2_new = coe_c / coe_a;
        double coe_3_new = (coe_d - target_timecost) / coe_a;
        int flag = SolveCubicEquation(coe_1_new, coe_2_new, coe_3_new, t_sol);
        //        cout << t_sol[0] << " & " << t_sol[1] << " & " << t_sol[2] << endl;
        if (flag == 1)
            return ceil(t_sol[0]);
        else if (flag == 2) {
            if (t_sol[0] < t_sol[1] && t_sol[0] > 0)
                return ceil(t_sol[0]);
            else
                return ceil(t_sol[1]);
        }
        else {
            if (t_sol[0] < t_sol[1] && t_sol[0] < t_sol[2] && t_sol[0] > 0)
                return ceil(t_sol[0]);
            else if (t_sol[1] < t_sol[0] && t_sol[1] < t_sol[2] && t_sol[1] > 0)
                return ceil(t_sol[1]);
            else
                return ceil(t_sol[2]);
        }
    }
}


void Optimizer::LocalBundleAdjustment(KeyFrame *pKF, bool *pbStopFlag, Map *pMap,
                                      MappingLog & time_log, BudgetPredictParam * param_)
{
#ifdef GOOD_GRAPH_TIME_LOGGING
    time_log.setGGTimerZero();
    //
    arma::wall_clock timer;
    timer.tic();
#endif

    list<KeyFrame *> lLocalKeyFrames;
    // create the optimizer object in SLAM++
#if defined ENABLE_STEREO_SLAMPP || defined ENABLE_QUATERNION
    Eigen::Matrix<double, 12, 1> pose_camera_;
#else
    Eigen::Matrix<double, 11, 1> pose_camera_;
#endif
    Eigen::Vector3d pose_lmk_;
    CBAOptimizer good_grapher(false);
    // NOTE
    // make sure the first index starts from 0
    std::unordered_map<size_t, KeyFrame *> hSLAMID2KF;
    std::unordered_map<KeyFrame *, size_t> hKF2SLAMID;
    std::unordered_map<MapPoint *, size_t> hMPt2SLAMID;
    //
    std::vector<size_t> idx_reserve_; // , idx_poses_;
    size_t slam_idx = 0;
    //    size_t lmk_id_offset = pMap->GetMaxKFid() + 100;

#ifdef ENABLE_ANTICIPATION_IN_GRAPH
    // include virtual KF
    std::vector<KeyFrame *> virtualKFs;
    if (!pKF->mvTrel.empty()) {
        size_t Npred = pKF->mvTrel.size();
        cv::Mat Tcw_base = pKF->GetPose(), Tcw_tmp;
        //
        for (size_t i=0; i<Npred; ++i) {
            Tcw_tmp = pKF->mvTrel[i] * Tcw_base;
            virtualKFs.push_back(new KeyFrame(pKF->mTimeStamp + double(i+1) * 0.05 * VIRTUAL_FRAME_STEP,
                                              Tcw_tmp, pKF->fx, pKF->fy, pKF->cx, pKF->cy, pKF->mb));
#ifdef ENABLE_STEREO_SLAMPP
            Optimizer::convertSKF2TVertex(virtualKFs.back(), pose_camera_);
            good_grapher.Add_SCamVertex(slam_idx, pose_camera_);
#else
            Optimizer::convertKF2TVertex(virtualKFs.back(), pose_camera_);
            good_grapher.Add_CamVertex(slam_idx, pose_camera_);
#endif
            idx_reserve_.push_back(slam_idx);
            slam_idx ++;
        }
    }
#endif

    // include most recent actual KF
    size_t root_kf_idx = slam_idx;
    lLocalKeyFrames.push_back(pKF);
    hSLAMID2KF.insert({slam_idx, pKF});
    hKF2SLAMID.insert({pKF, slam_idx});
    pKF->mnBALocalForKFCand = pKF->mnId;
#ifdef ENABLE_STEREO_SLAMPP
    Optimizer::convertSKF2TVertex(pKF, pose_camera_);
    good_grapher.Add_SCamVertex(slam_idx, pose_camera_);
#else
    Optimizer::convertKF2TVertex(pKF, pose_camera_);
    good_grapher.Add_CamVertex(slam_idx, pose_camera_);
#endif
    //    std::cout << slam_idx << "; " << pose_camera_ << std::endl;
    //
    //    idx_poses_.push_back(slam_idx);
    // set the last camera state un-removable
    idx_reserve_.push_back(slam_idx);
    slam_idx ++;
#ifdef GOOD_GRAPH_TIME_LOGGING
    time_log.time_gg_insert_vertex += timer.toc();
#endif

    // Local KeyFrames: First Breath Search from Current Keyframe
    const vector<KeyFrame *> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
    for (int i = 0, iend = vNeighKFs.size(); i < iend; i++)
    {
        KeyFrame *pKFi = vNeighKFs[i];
        pKFi->mnBALocalForKFCand = pKF->mnId;
        if (!pKFi->isBad()) {
            lLocalKeyFrames.push_back(pKFi);
#ifdef GOOD_GRAPH_TIME_LOGGING
            timer.tic();
#endif
            hSLAMID2KF.insert({slam_idx, pKFi});
            hKF2SLAMID.insert({pKFi, slam_idx});
            // Eigen::Matrix<double, 11, 1> &v_cam_state
#ifdef ENABLE_STEREO_SLAMPP
            Optimizer::convertSKF2TVertex(pKFi, pose_camera_);
            good_grapher.Add_SCamVertex(slam_idx, pose_camera_);
#else
            Optimizer::convertKF2TVertex(pKFi, pose_camera_);
            good_grapher.Add_CamVertex(slam_idx, pose_camera_);
#endif
            //            idx_poses_.push_back(slam_idx);
            slam_idx ++;
#ifdef GOOD_GRAPH_TIME_LOGGING
            time_log.time_gg_insert_vertex += timer.toc();
#endif

            // limit the size of KF pool to 60
            if (slam_idx == GOOD_GRAPH_KF_MAXSZ)
                break ;
        }
    }
    //
    time_log.num_free_KF = lLocalKeyFrames.size();

    // Local MapPoints seen in Local KeyFrames
    list<MapPoint *> lLocalMapPoints;
    for (const auto & lit : hSLAMID2KF)
    {
        vector<MapPoint *> vpMPs = lit.second->GetMapPointMatches();
        //        for (vector<MapPoint *>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; vit++)
        for (auto & vit : vpMPs)
        {
            MapPoint *pMP = vit;
            if (pMP) {
                if (!pMP->isBad()) {
                    if (pMP->mnBALocalForKFCand != pKF->mnId)
                    {
                        // insert newly found lmk
                        lLocalMapPoints.push_back(pMP);
#ifdef GOOD_GRAPH_TIME_LOGGING
                        timer.tic();
#endif
                        hMPt2SLAMID.insert({pMP, slam_idx});
                        pMP->mnBALocalForKFCand = pKF->mnId;
                        // Eigen::Vector3d &v_position
                        Optimizer::convertMP2TVertex(pMP, pose_lmk_);
                        good_grapher.Add_XYZVertex(slam_idx, pose_lmk_);
                        slam_idx ++;
#ifdef GOOD_GRAPH_TIME_LOGGING
                        time_log.time_gg_insert_vertex += timer.toc();
#endif
                    }
                }
            }
        }
    }
    //
    time_log.num_Point = lLocalMapPoints.size();

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    list<KeyFrame *> lFixedCameras;
    size_t neg_slam_idx = -1;
    for (const auto & lit : lLocalMapPoints)
    {
        map<KeyFrame *, size_t> observations = lit->GetObservations();
        for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
        {
            KeyFrame *pKFi = mit->first;

            if (pKFi->mnBALocalForKFCand != pKF->mnId && pKFi->mnBAFixedForKFCand != pKF->mnId)
            {
                pKFi->mnBAFixedForKFCand = pKF->mnId;
                if (!pKFi->isBad()) {
                    lFixedCameras.push_back(pKFi);
#ifdef GOOD_GRAPH_TIME_LOGGING
                    timer.tic();
#endif
                    hKF2SLAMID.insert({pKFi, neg_slam_idx});
#ifdef ENABLE_STEREO_SLAMPP
                    Optimizer::convertSKF2TVertex(pKFi, pose_camera_);
                    good_grapher.Add_SCamVertex_Fixed(slam_idx, pose_camera_);
#else
                    Optimizer::convertKF2TVertex(pKFi, pose_camera_);
                    good_grapher.Add_CamVertex_Fixed(neg_slam_idx, pose_camera_);
#endif
                    neg_slam_idx --;
#ifdef GOOD_GRAPH_TIME_LOGGING
                    time_log.time_gg_insert_vertex += timer.toc();
#endif
                }
            }
        }
    }

    time_log.num_fixed_KF = lFixedCameras.size();

    // insert edges
    size_t root_matching_num = 0;
    //    for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++)
    for (const auto & lit : lLocalMapPoints)
    {
        MapPoint *pMP = lit;
        const map<KeyFrame *, size_t> observations = pMP->GetObservations();

        //Set edges
        for (map<KeyFrame *, size_t>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
        {
            KeyFrame *pKFi = mit->first;

            if (!pKFi->isBad())
            {
                const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];

#ifdef GOOD_GRAPH_TIME_LOGGING
                timer.tic();
#endif

#ifdef ENABLE_STEREO_SLAMPP
                // Monocular observation
                if (pKFi->mvuRight[mit->second] < 0)
                {
                    // estimate disparity
                    double disp_ = 0;
                    Eigen::Vector3d v_observation;
                    v_observation << kpUn.pt.x, kpUn.pt.y, disp_;

                    // size_t n_xyz_vertex_id, size_t n_cam_vertex_id,
                    // const Eigen::Vector2d &v_observation, const Eigen::Matrix2d &t_information
                    size_t mptID;
                    auto it_tmp_1 = hMPt2SLAMID.find(pMP);
                    if (it_tmp_1 != hMPt2SLAMID.end())
                        mptID = it_tmp_1->second;
                    else {
                        std::cout << "invalid hash query in hMPt2SLAMID" << std::endl;
                        continue ;
                    }
                    //
                    size_t kfID;
                    auto it_tmp_2 = hKF2SLAMID.find(pKFi);
                    if (it_tmp_2 != hKF2SLAMID.end())
                        kfID = it_tmp_2->second;
                    else {
                        std::cout << "invalid hash query in hKF2SLAMID" << std::endl;
                        continue ;
                    }
                    //                size_t mptID = hMPt2SLAMID[pMP];
                    //                size_t kfID = hKF2SLAMID[pKFi];
                    Eigen::Matrix3d info_mat_ = Eigen::Matrix3d::Identity() * pKFi->mvInvLevelSigma2[kpUn.octave];
                    // set extreme uncertainty at disparity dimension
                    info_mat_(3,3) = DBL_MIN;
                    good_grapher.Add_P2SC3DEdge(mptID, kfID, v_observation, info_mat_);
                }
                else // Stereo observation
                {
                    Eigen::Vector3d v_observation;
                    v_observation << kpUn.pt.x, kpUn.pt.y, pKFi->mvuRight[mit->second];

                    // size_t n_xyz_vertex_id, size_t n_cam_vertex_id,
                    // const Eigen::Vector3d &v_observation, const Eigen::Matrix3d &t_information
                    size_t mptID;
                    auto it_tmp_1 = hMPt2SLAMID.find(pMP);
                    if (it_tmp_1 != hMPt2SLAMID.end())
                        mptID = it_tmp_1->second;
                    else {
                        std::cout << "invalid hash query in hMPt2SLAMID" << std::endl;
                        continue ;
                    }
                    //
                    size_t kfID;
                    auto it_tmp_2 = hKF2SLAMID.find(pKFi);
                    if (it_tmp_2 != hKF2SLAMID.end())
                        kfID = it_tmp_2->second;
                    else {
                        std::cout << "invalid hash query in hKF2SLAMID" << std::endl;
                        continue ;
                    }
                    good_grapher.Add_P2SC3DEdge(mptID, kfID, v_observation,
                                                Eigen::Matrix3d::Identity() * pKFi->mvInvLevelSigma2[kpUn.octave]);
                }
#else
                Eigen::Vector2d v_observation;
                //                v_observation << kpUn.pt.x - pKFi->cx, kpUn.pt.y - pKFi->cy;
                v_observation << kpUn.pt.x, kpUn.pt.y;

                // size_t n_xyz_vertex_id, size_t n_cam_vertex_id,
                // const Eigen::Vector2d &v_observation, const Eigen::Matrix2d &t_information
                size_t mptID;
                auto it_tmp_1 = hMPt2SLAMID.find(pMP);
                if (it_tmp_1 != hMPt2SLAMID.end())
                    mptID = it_tmp_1->second;
                else {
                    std::cout << "invalid hash query in hMPt2SLAMID" << std::endl;
                    continue ;
                }
                //
                size_t kfID;
                auto it_tmp_2 = hKF2SLAMID.find(pKFi);
                if (it_tmp_2 != hKF2SLAMID.end())
                    kfID = it_tmp_2->second;
                else {
                    std::cout << "invalid hash query in hKF2SLAMID" << std::endl;
                    continue ;
                }
                //                size_t mptID = hMPt2SLAMID[pMP];
                //                size_t kfID = hKF2SLAMID[pKFi];
                good_grapher.Add_P2C3DEdge(mptID, kfID, v_observation,
                                           Eigen::Matrix2d::Identity() * pKFi->mvInvLevelSigma2[kpUn.octave]);
                //
                if (kfID == root_kf_idx) {
                    root_matching_num ++;
                }
#endif

#ifdef GOOD_GRAPH_TIME_LOGGING
                time_log.time_gg_insert_vertex += timer.toc();
#endif
                //
            }
        }
    }

#ifdef ENABLE_ANTICIPATION_IN_GRAPH
    //
#ifdef GOOD_GRAPH_TIME_LOGGING
    timer.tic();
#endif
    // check visibility in virtual KFs
    std::vector<size_t> visiblePtNum;
    //    visiblePtNum.push_back(pKF->GetMatchNum());
    if (pKF->mNumVisibleMpt > 0)
        visiblePtNum.push_back(pKF->mNumVisibleMpt);
    else
        visiblePtNum.push_back(time_log.num_Point);
    //
    size_t vkf_cnt = 0;
    for (auto & vfit : virtualKFs)
    {
        size_t vedge_cnt = 0;
        KeyFrame *vkf = vfit;
        cv::Mat Tcw_tmp = vkf->GetPose();
        for (const auto & lit : lLocalMapPoints)
        {
            MapPoint *pMP = lit;
            // map to frame projection
            cv::Mat P = pMP->GetWorldPos();
            cv::Mat Pc = Tcw_tmp.rowRange(0, 3).colRange(0, 3) * P + Tcw_tmp.rowRange(0, 3).col(3);
            float &PcX = Pc.at<float>(0);
            float &PcY = Pc.at<float>(1);
            float &PcZ = Pc.at<float>(2);
            // Check positive depth
            if (PcZ < 0.0f)
                continue ;

            // Project in image and check it is not outside
            float invz = 1.0f / PcZ;
            float u = vkf->fx * PcX * invz + vkf->cx;
            float v = vkf->fy * PcY * invz + vkf->cy;
            if (u < Frame::mnMinX || u > Frame::mnMaxX)
                continue ;
            if (v < Frame::mnMinY || v > Frame::mnMaxY)
                continue ;

            // Check distance is in the scale invariance region of the MapPoint
            float maxDistance = pMP->GetMaxDistanceInvariance();
            float minDistance = pMP->GetMinDistanceInvariance();
            cv::Mat PO = P - vkf->GetOw();
            float dist = cv::norm(PO);
            if (dist < minDistance || dist > maxDistance)
                continue ;

            // Check viewing angle
            cv::Mat Pn = pMP->GetNormal();
            if (PO.dot(Pn) / dist < 0.5)
                continue ;

            // Predict scale in the image
            int nPredictedLevel = pMP->PredictScale(dist, pKF);

            // Found a potentially visible point in vkf
            Eigen::Vector2d v_observation;
            v_observation << u, v;
            size_t mptID;
            auto it_tmp_1 = hMPt2SLAMID.find(pMP);
            if (it_tmp_1 != hMPt2SLAMID.end())
                mptID = it_tmp_1->second;
            else {
                std::cout << "invalid hash query in hMPt2SLAMID" << std::endl;
                continue ;
            }
            //
            good_grapher.Add_P2C3DEdge(mptID, vkf_cnt, v_observation,
                                       Eigen::Matrix2d::Identity() * pKF->mvInvLevelSigma2[nPredictedLevel]);
            vedge_cnt ++;
        }
        //
        visiblePtNum.push_back(vedge_cnt);
        vkf_cnt ++;
    }

#ifdef GOOD_GRAPH_TIME_LOGGING
    time_log.time_gg_prediction = timer.toc();
#endif
    //
#endif

#ifdef ENABLE_ANTICIPATION_IN_BUDGET
    // TODO
    // estimate the budget for BA
    // a minimum budget of 100ms is allocated
    size_t szGoodGraph = GOOD_GRAPH_KF_THRES;
    if (param_ != NULL) {
        //
        // std::cout << "param_->budget_per_frame = " << param_->budget_per_frame * 1000.0f << std::endl;

        // set minimum amount of local BA to 100 ms
        double min_LBA_budget = 0.10f;
        // for EuRoC benchmark
        // set maximum amount of local BA to 800 ms
        double max_LBA_budget = 0.80f;
        //
        // // for FPV benchmark
        // // set maximum amount of local BA to 330 ms
        // double max_LBA_budget = 0.33f;

        double budgetLBA = min_LBA_budget;

        if (param_->match_ratio_log.size() < NUM_HISTORICAL_BUDGET) {
            // no fully loaded yet
            param_->match_ratio_log.push_back(double(root_matching_num) / double(pKF->mNumVisibleMpt));
            param_->match_ratio_idx ++;
        }
        else {
            // start ring buffering
            param_->match_ratio_log[param_->match_ratio_idx % NUM_HISTORICAL_BUDGET] =
                    double(root_matching_num) / double(pKF->mNumVisibleMpt);
            param_->match_ratio_idx ++;
        }
        double matchRatio = std::accumulate(param_->match_ratio_log.begin(),
                                            param_->match_ratio_log.end(),
                                            0.0) / double( param_->match_ratio_log.size() );
        //        double matchRatio = double(root_matching_num) / double(num_Point);

#ifdef DEBUG_VERBOSE
        std::cout << "num of map visible: " << pKF->mNumVisibleMpt
                  << "; num of matching found: " << root_matching_num
                  << "; matching ratio: " << matchRatio << std::endl;
#endif
        size_t vidx = 0;
        // EuRoC & FPV
        // size_t minVisibleNum = ceil(double(pKF->N) * 0.30f / 0.35f);
        // Gazebo
        // size_t minVisibleNum = ceil(double(pKF->N) * 0.20f);
        size_t minVisibleNum = ceil(double(pKF->N) * 0.40f);
#ifdef DEBUG_VERBOSE
        cout << "visiblePtNum: " ;
#endif
        while (vidx <= vkf_cnt) {
#ifdef DEBUG_VERBOSE
            cout << visiblePtNum[vidx] << ", ";
#endif
            if (visiblePtNum[vidx] <= minVisibleNum) {
                break ;
            }
            ++ vidx;
        }
#ifdef DEBUG_VERBOSE
        cout << "; minVisibleNum = " << minVisibleNum << endl;
#endif

        // set budget till reaching the minimum visible threshold, in seconds
        if (vidx == 0) {
            // keep the minimum budget
            budgetLBA = min_LBA_budget;
        }
        else if (vidx <= vkf_cnt) {
            //            budgetLBA = min( max( budgetLBA,
            //                                  double(vidx * VIRTUAL_FRAME_STEP) * param_->budget_per_frame
            //                                  ), MAX_LOCAL_BA_BUDGET );
            if (visiblePtNum[vidx-1] <= visiblePtNum[vidx])
                budgetLBA = double(VIRTUAL_FRAME_STEP * vidx) * param_->budget_per_frame; // MAX_LOCAL_BA_BUDGET;
            else
                budgetLBA = min( max( budgetLBA,
                                      double(VIRTUAL_FRAME_STEP) * param_->budget_per_frame *
                                      double(visiblePtNum[vidx-1] - minVisibleNum) / double(visiblePtNum[vidx-1] - visiblePtNum[vidx])
                        ), max_LBA_budget );
        }
        else {
            // extrapolate to further horizon
            //            budgetLBA = min( max( budgetLBA,
            //                                  double(vkf_cnt * VIRTUAL_FRAME_STEP) *
            //                                  param_->budget_per_frame *
            //                                  (1.0 + 1.0 / (1.0 - double(visiblePtNum[vkf_cnt-1] - minVisibleNum) /
            //                                   double(visiblePtNum[vkf_cnt] - minVisibleNum)))
            //                             ), MAX_LOCAL_BA_BUDGET );
            if (visiblePtNum[vkf_cnt-1] <= visiblePtNum[vkf_cnt])
                budgetLBA = max_LBA_budget;
            else
                budgetLBA = min( max( budgetLBA,
                                      double(VIRTUAL_FRAME_STEP) * param_->budget_per_frame *
                                      double(visiblePtNum[vkf_cnt-1] - minVisibleNum) / double(visiblePtNum[vkf_cnt-1] - visiblePtNum[vkf_cnt])
                        ), max_LBA_budget );
        }
        //        cout << "budgetLBA = " << budgetLBA << endl;

#ifdef DEBUG_VERBOSE
        std::cout << "coe_a = " << param_->coe_a
                  << "; coe_b = " << param_->coe_b
                  << "; coe_c = " << param_->coe_c
                  << "; coe_d = " << param_->coe_d
                  << "; budgetLBA (ms) = " << budgetLBA * 1000.0 << std::endl;
#endif

#ifdef GOOD_GRAPH_TIME_LOGGING
        time_log.time_gg_lba_budget = budgetLBA;
#endif

        // estimate the size of good graph
        szGoodGraph = max(3, Optimizer::estimateKFNum(param_->coe_a, param_->coe_b,
                                                      param_->coe_c, param_->coe_d,
                                                      budgetLBA * 1000.0)) + VIRTUAL_FRAME_NUM;

#ifdef DEBUG_VERBOSE
        std::cout << "vidx = " << vidx << "; budgetLBA (ms) = " << budgetLBA * 1000.0 << "; szGoodGraph = " << szGoodGraph << std::endl;
#endif
    }
    if (szGoodGraph < time_log.num_free_KF + VIRTUAL_FRAME_NUM) {
#else
    // simply fix the size of good graph
    if (time_log.num_free_KF > GOOD_GRAPH_KF_THRES) {
#ifdef ENABLE_ANTICIPATION_IN_GRAPH
        size_t szGoodGraph = GOOD_GRAPH_KF_THRES + VIRTUAL_FRAME_NUM; // num_free_KF/2;
#else
        size_t szGoodGraph = GOOD_GRAPH_KF_THRES; // num_free_KF/2;
#endif

#endif
        // start good graph
        // printf("Good Graph Triggered -------------\n");
        // printf("total num. of camera poses = %d, lmk = %d, fixed poses = %d\n", num_free_KF, num_Point, num_fixed_KF);
        //        printf("target num. of camera poses = %d\n", szGoodGraph);
        //    optimizer.Dump_Graph("graph_orig.graph");

        // DEBUG
        //        good_grapher.Optimize();
        //        good_grapher.Show_Stats();
        //        good_grapher.Dump_Graph("graph_sub.graph");
        //        good_grapher.Dump_State("solution.txt");

        // perfrom subgraph selection
        // reduce the graph to meet cardinality constr. szGoodGraph
        double logdt = good_grapher.Find_Subgraph(szGoodGraph, 1, idx_reserve_);
        // std::cout << "logDet(S) = " << logdt << std::endl;
        //        good_grapher.Show_Stats();
        good_grapher.Dump_TimeLog(time_log.time_gg_jacobian, time_log.time_gg_preproc, time_log.time_gg_schur, time_log.time_gg_rnd_query,
                                  time_log.time_gg_permute, time_log.time_gg_cholesky, time_log.time_gg_postproc);
        //        mTimeJacobain = good_grapher.GetOptimizer()->r_Solver().m_f_pre_time;
        //        mTimeSchur = good_grapher.GetOptimizer()->r_Solver().m_f_schur_time;
        //        mTimePerm = good_grapher.GetOptimizer()->r_Solver().m_f_slice_time;
        //        mTimeCholesky = good_grapher.GetOptimizer()->r_Solver().m_f_chol_time;
        //        mTimePost = good_grapher.GetOptimizer()->r_Solver().m_f_post_time;

#ifdef GOOD_GRAPH_TIME_LOGGING
        timer.tic();
#endif
        // update vertex and edge list to be used in g2o
        lLocalKeyFrames.clear();
        for (const auto & iter : idx_reserve_) {
            //
            auto it_tmp = hSLAMID2KF.find(iter);
            if (it_tmp != hSLAMID2KF.end()) {
                it_tmp->second->mnBALocalForKF = pKF->mnId;
                lLocalKeyFrames.push_back(it_tmp->second);
            }
            else {
                //                std::cout << "invalid hash query in hSLAMID2KF" << std::endl;
                continue ;
            }
            //
            //            hSLAMID2KF[iter]->mnBALocalForKF = pKF->mnId;
            //            lLocalKeyFrames.push_back(hSLAMID2KF[iter]);
        }
        time_log.gg_num_free_KF = lLocalKeyFrames.size();

        // Local MapPoints seen in Local KeyFrames
        lLocalMapPoints.clear();
        for (const auto & lit : lLocalKeyFrames)
        {
            vector<MapPoint *> vpMPs = lit->GetMapPointMatches();
            //        for (vector<MapPoint *>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; vit++)
            for (auto & vit : vpMPs)
            {
                MapPoint *pMP = vit;
                if (pMP) {
                    if (!pMP->isBad()) {
                        if (pMP->mnBALocalForKF != pKF->mnId)
                        {
                            // insert newly found lmk
                            lLocalMapPoints.push_back(pMP);
                            pMP->mnBALocalForKF = pKF->mnId;
                        }
                    }
                }
            }
        }
        //
        time_log.gg_num_Point = lLocalMapPoints.size();

        // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
        lFixedCameras.clear();
        //    for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++)
        for (const auto & lit : lLocalMapPoints)
        {
            map<KeyFrame *, size_t> observations = lit->GetObservations();
            for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
            {
                KeyFrame *pKFi = mit->first;

                if (pKFi->mnBALocalCount >= GOOD_GRAPH_FIXED_THRES && pKFi->mnBALocalForKFCand != pKF->mnId &&
                        pKFi->mnBALocalForKF != pKF->mnId &&
                        pKFi->mnBAFixedForKF != pKF->mnId)
                {
                    pKFi->mnBAFixedForKF = pKF->mnId;
                    if (!pKFi->isBad()) {
                        lFixedCameras.push_back(pKFi);
                    }
                }
            }
        }
        //
        time_log.gg_num_fixed_KF = lFixedCameras.size();

        // printf("subset num. of camera poses = %d, lmk = %d, fixed poses = %d\n", num_free_KF, num_Point, num_fixed_KF);

#ifdef GOOD_GRAPH_TIME_LOGGING
        time_log.time_gg_optimization += timer.toc();
#endif
    }

#ifdef GOOD_GRAPH_TIME_LOGGING
    timer.tic();
#endif
    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if (pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    unsigned long maxKFid = 0;

    // Set Local KeyFrame vertices
    for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++)
    {
        KeyFrame *pKFi = *lit;
        g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(pKFi->mnId == 0);
        optimizer.addVertex(vSE3);
        if (pKFi->mnId > maxKFid)
            maxKFid = pKFi->mnId;
        //
        pKFi->mnBALocalForKF = pKF->mnId;
        pKFi->mnBALocalCount ++;
    }

    // Set Fixed KeyFrame vertices
    for (list<KeyFrame *>::iterator lit = lFixedCameras.begin(), lend = lFixedCameras.end(); lit != lend; lit++)
    {
        KeyFrame *pKFi = *lit;
        g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(true);
        optimizer.addVertex(vSE3);
        if (pKFi->mnId > maxKFid)
            maxKFid = pKFi->mnId;
        // duplicate ???
        pKFi->mnBAFixedForKF = pKF->mnId;
    }

    // Set MapPoint vertices
    const int nExpectedSize = (lLocalKeyFrames.size() + lFixedCameras.size()) * lLocalMapPoints.size();

    vector<g2o::EdgeSE3ProjectXYZ *> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    vector<KeyFrame *> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    vector<MapPoint *> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    vector<g2o::EdgeStereoSE3ProjectXYZ *> vpEdgesStereo;
    vpEdgesStereo.reserve(nExpectedSize);

    vector<KeyFrame *> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);

    vector<MapPoint *> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);

    const float thHuberMono = sqrt(5.991);
    const float thHuberStereo = sqrt(7.815);

    for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++)
    {
        MapPoint *pMP = *lit;
        g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        int id = pMP->mnId + maxKFid + 1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        const map<KeyFrame *, size_t> observations = pMP->GetObservations();

        //Set edges
        for (map<KeyFrame *, size_t>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
        {
            KeyFrame *pKFi = mit->first;

            if ( !pKFi->isBad() && (pKFi->mnBALocalForKF == pKF->mnId || pKFi->mnBAFixedForKF == pKF->mnId) )
            {
                const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];

                // Monocular observation
                if (pKFi->mvuRight[mit->second] < 0)
                {
                    Eigen::Matrix<double, 2, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    g2o::EdgeSE3ProjectXYZ *e = new g2o::EdgeSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;

                    optimizer.addEdge(e);

                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);
                }
                else // Stereo observation
                {
                    Eigen::Matrix<double, 3, 1> obs;
                    const float kp_ur = pKFi->mvuRight[mit->second];
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    g2o::EdgeStereoSE3ProjectXYZ *e = new g2o::EdgeStereoSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
                    e->setInformation(Info);

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;
                    e->bf = pKFi->mbf;

                    optimizer.addEdge(e);

                    vpEdgesStereo.push_back(e);
                    vpEdgeKFStereo.push_back(pKFi);
                    vpMapPointEdgeStereo.push_back(pMP);
                }
            }
        }
    }

    if (pbStopFlag) {
        if (*pbStopFlag) {
#ifdef GOOD_GRAPH_TIME_LOGGING
            time_log.time_gg_optimization += timer.toc();
#endif
            return;
        }
    }

    optimizer.initializeOptimization();

#ifdef GROUND_TRUTH_GEN_MODE
    optimizer.optimize(15);
#else
    optimizer.optimize(5);
#endif

    bool bDoMore = true;

    if (pbStopFlag)
        if (*pbStopFlag)
            bDoMore = false;

    if (bDoMore)
    {
        // Check inlier observations
        for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
        {
            g2o::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
            MapPoint *pMP = vpMapPointEdgeMono[i];

            if (pMP->isBad())
                continue;

            if (e->chi2() > 5.991 || !e->isDepthPositive())
            {
                e->setLevel(1);
            }

            e->setRobustKernel(0);
        }

        for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
        {
            g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
            MapPoint *pMP = vpMapPointEdgeStereo[i];

            if (pMP->isBad())
                continue;

            if (e->chi2() > 7.815 || !e->isDepthPositive())
            {
                e->setLevel(1);
            }

            e->setRobustKernel(0);
        }

        // Optimize again without the outliers

        optimizer.initializeOptimization(0);

#ifdef GROUND_TRUTH_GEN_MODE
        optimizer.optimize(20);
#else
        optimizer.optimize(10);
#endif
    }

    vector<pair<KeyFrame *, MapPoint *>> vToErase;
    vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());

    // Check inlier observations
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
    {
        g2o::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
        MapPoint *pMP = vpMapPointEdgeMono[i];

        if (pMP->isBad())
            continue;

        if (e->chi2() > 5.991 || !e->isDepthPositive())
        {
            KeyFrame *pKFi = vpEdgeKFMono[i];
            vToErase.push_back(make_pair(pKFi, pMP));
        }
    }

    for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
    {
        g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
        MapPoint *pMP = vpMapPointEdgeStereo[i];

        if (pMP->isBad())
            continue;

        if (e->chi2() > 7.815 || !e->isDepthPositive())
        {
            KeyFrame *pKFi = vpEdgeKFStereo[i];
            vToErase.push_back(make_pair(pKFi, pMP));
        }
    }

    // Get Map Mutex
    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    if (!vToErase.empty())
    {
        for (size_t i = 0; i < vToErase.size(); i++)
        {
            KeyFrame *pKFi = vToErase[i].first;
            MapPoint *pMPi = vToErase[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }
    }

    // Recover optimized data

    //Keyframes
    for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++)
    {
        KeyFrame *pKF = *lit;
        g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pKF->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        pKF->SetPose(Converter::toCvMat(SE3quat));
    }

    //Points
    for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++)
    {
        MapPoint *pMP = *lit;
        g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(pMP->mnId + maxKFid + 1));
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
        pMP->UpdateNormalAndDepth();
    }

#ifdef GOOD_GRAPH_TIME_LOGGING
    time_log.time_gg_optimization += timer.toc();
#endif

}

#else
//
void Optimizer::LocalBundleAdjustment(KeyFrame *pKF, bool *pbStopFlag, Map *pMap,
                                      MappingLog & time_log)
{
#ifdef GOOD_GRAPH_TIME_LOGGING
    time_log.setGGTimerZero();
    //
    arma::wall_clock timer;
    timer.tic();
#endif

    // Local KeyFrames: First Breath Search from Current Keyframe
    list<KeyFrame *> lLocalKeyFrames;

    lLocalKeyFrames.push_back(pKF);
    pKF->mnBALocalForKF = pKF->mnId;

    const vector<KeyFrame *> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
    for (int i = 0, iend = vNeighKFs.size(); i < iend; i++)
    {
        KeyFrame *pKFi = vNeighKFs[i];
        pKFi->mnBALocalForKF = pKF->mnId;
        if (!pKFi->isBad()) {
            lLocalKeyFrames.push_back(pKFi);
        }
    }

    //
    time_log.num_free_KF = lLocalKeyFrames.size();

#ifdef ENABLE_SLIDING_WINDOW_FILTER
    // sort KF according to timestamp; keep recent-N ones
    if (time_log.num_free_KF > GOOD_GRAPH_KF_THRES) {
        //        sort(lLocalKeyFrames.begin(), lLocalKeyFrames.end(), KeyFrame::timeStampComp);
        lLocalKeyFrames.sort(KeyFrame::timeStampComp);
        while (time_log.num_free_KF > GOOD_GRAPH_KF_THRES) {
            (*lLocalKeyFrames.begin())->mnBALocalForKF = pKF->mnId - 1;
            lLocalKeyFrames.pop_front();
            time_log.num_free_KF --;
        }
        assert(lLocalKeyFrames.size() == GOOD_GRAPH_KF_THRES && (lLocalKeyFrames.back())->mnId == pKF->mnId);
    }
#elif defined ENABLE_COVIS_GRAPH
    // sort KF accoding to co-vis weight; keep top-N ones
    if (time_log.num_free_KF > GOOD_GRAPH_KF_THRES) {
        while (time_log.num_free_KF > GOOD_GRAPH_KF_THRES) {
            lLocalKeyFrames.back()->mnBALocalForKF = pKF->mnId - 1;
            lLocalKeyFrames.pop_back();
            time_log.num_free_KF --;
        }
        assert(lLocalKeyFrames.size() == GOOD_GRAPH_KF_THRES && (*lLocalKeyFrames.begin())->mnId == pKF->mnId);
    }
#endif

    // Local MapPoints seen in Local KeyFrames
    list<MapPoint *> lLocalMapPoints;
    for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++)
    {
        vector<MapPoint *> vpMPs = (*lit)->GetMapPointMatches();
        for (vector<MapPoint *>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; vit++)
        {
            MapPoint *pMP = *vit;
            if (pMP) {
                if (!pMP->isBad()) {
                    if (pMP->mnBALocalForKF != pKF->mnId)
                    {
                        lLocalMapPoints.push_back(pMP);
                        pMP->mnBALocalForKF = pKF->mnId;
                    }
                }
            }
        }
    }
    //
    time_log.num_Point = lLocalMapPoints.size();

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    list<KeyFrame *> lFixedCameras;
    for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++)
    {
        map<KeyFrame *, size_t> observations = (*lit)->GetObservations();
        for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
        {
            KeyFrame *pKFi = mit->first;

            if (pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId)
            {
#if defined ENABLE_SLIDING_WINDOW_FILTER || defined ENABLE_COVIS_GRAPH
                if (pKFi->mnBALocalCount >= GOOD_GRAPH_FIXED_THRES)
                {
#endif
                pKFi->mnBAFixedForKF = pKF->mnId;
                if (!pKFi->isBad()) {
                    lFixedCameras.push_back(pKFi);
                }
#if defined ENABLE_SLIDING_WINDOW_FILTER || defined ENABLE_COVIS_GRAPH
                }
#endif
            }
        }
    }
    //
    time_log.num_fixed_KF = lFixedCameras.size();

    // printf("subset num. of camera poses = %d, lmk = %d, fixed poses = %d\n", num_free_KF, num_Point, num_fixed_KF);

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if (pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    unsigned long maxKFid = 0;

    // Set Local KeyFrame vertices
    for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++)
    {
        KeyFrame *pKFi = *lit;
        g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(pKFi->mnId == 0);
        optimizer.addVertex(vSE3);
        if (pKFi->mnId > maxKFid)
            maxKFid = pKFi->mnId;
        //
        pKFi->mnBALocalCount ++;
    }

    // Set Fixed KeyFrame vertices
    for (list<KeyFrame *>::iterator lit = lFixedCameras.begin(), lend = lFixedCameras.end(); lit != lend; lit++)
    {
        KeyFrame *pKFi = *lit;
        g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(true);
        optimizer.addVertex(vSE3);
        if (pKFi->mnId > maxKFid)
            maxKFid = pKFi->mnId;
    }

    // Set MapPoint vertices
    const int nExpectedSize = (lLocalKeyFrames.size() + lFixedCameras.size()) * lLocalMapPoints.size();

    vector<g2o::EdgeSE3ProjectXYZ *> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    vector<KeyFrame *> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    vector<MapPoint *> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    vector<g2o::EdgeStereoSE3ProjectXYZ *> vpEdgesStereo;
    vpEdgesStereo.reserve(nExpectedSize);

    vector<KeyFrame *> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);

    vector<MapPoint *> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);

    const float thHuberMono = sqrt(5.991);
    const float thHuberStereo = sqrt(7.815);

    for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++)
    {
        MapPoint *pMP = *lit;
        g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        int id = pMP->mnId + maxKFid + 1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        const map<KeyFrame *, size_t> observations = pMP->GetObservations();

        //Set edges
        for (map<KeyFrame *, size_t>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
        {
            KeyFrame *pKFi = mit->first;

            if (!pKFi->isBad())
            {
                const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];

                // Monocular observation
                if (pKFi->mvuRight[mit->second] < 0)
                {
                    Eigen::Matrix<double, 2, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    g2o::EdgeSE3ProjectXYZ *e = new g2o::EdgeSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;

                    optimizer.addEdge(e);
                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);
                }
                else // Stereo observation
                {
                    Eigen::Matrix<double, 3, 1> obs;
                    const float kp_ur = pKFi->mvuRight[mit->second];
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    g2o::EdgeStereoSE3ProjectXYZ *e = new g2o::EdgeStereoSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
                    e->setInformation(Info);

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;
                    e->bf = pKFi->mbf;

                    optimizer.addEdge(e);
                    vpEdgesStereo.push_back(e);
                    vpEdgeKFStereo.push_back(pKFi);
                    vpMapPointEdgeStereo.push_back(pMP);
                }
            }
        }
    }

    if (pbStopFlag) {
        if (*pbStopFlag) {
#ifdef GOOD_GRAPH_TIME_LOGGING
            time_log.time_gg_optimization += timer.toc();
#endif
            return;
        }
    }

    optimizer.initializeOptimization();

#ifdef GROUND_TRUTH_GEN_MODE
    optimizer.optimize(15);
#else
    optimizer.optimize(5);
#endif

    bool bDoMore = true;

    if (pbStopFlag)
        if (*pbStopFlag)
            bDoMore = false;

    if (bDoMore)
    {
        // Check inlier observations
        for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
        {
            g2o::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
            MapPoint *pMP = vpMapPointEdgeMono[i];

            if (pMP->isBad())
                continue;

            if (e->chi2() > 5.991 || !e->isDepthPositive())
            {
                e->setLevel(1);
            }

            e->setRobustKernel(0);
        }

        for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
        {
            g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
            MapPoint *pMP = vpMapPointEdgeStereo[i];

            if (pMP->isBad())
                continue;

            if (e->chi2() > 7.815 || !e->isDepthPositive())
            {
                e->setLevel(1);
            }

            e->setRobustKernel(0);
        }

        // Optimize again without the outliers

        optimizer.initializeOptimization(0);

#ifdef GROUND_TRUTH_GEN_MODE
        optimizer.optimize(20);
#else
        optimizer.optimize(10);
#endif

    }

    vector<pair<KeyFrame *, MapPoint *>> vToErase;
    vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());

    // Check inlier observations
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
    {
        g2o::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
        MapPoint *pMP = vpMapPointEdgeMono[i];

        if (pMP->isBad())
            continue;

        if (e->chi2() > 5.991 || !e->isDepthPositive())
        {
            KeyFrame *pKFi = vpEdgeKFMono[i];
            vToErase.push_back(make_pair(pKFi, pMP));
        }
    }

    for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
    {
        g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
        MapPoint *pMP = vpMapPointEdgeStereo[i];

        if (pMP->isBad())
            continue;

        if (e->chi2() > 7.815 || !e->isDepthPositive())
        {
            KeyFrame *pKFi = vpEdgeKFStereo[i];
            vToErase.push_back(make_pair(pKFi, pMP));
        }
    }

    // Get Map Mutex
    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    if (!vToErase.empty())
    {
        for (size_t i = 0; i < vToErase.size(); i++)
        {
            KeyFrame *pKFi = vToErase[i].first;
            MapPoint *pMPi = vToErase[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }
    }

    // Recover optimized data

    //Keyframes
    for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++)
    {
        KeyFrame *pKF = *lit;
        g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pKF->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        pKF->SetPose(Converter::toCvMat(SE3quat));
    }

    //Points
    for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++)
    {
        MapPoint *pMP = *lit;
        g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(pMP->mnId + maxKFid + 1));
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
        pMP->UpdateNormalAndDepth();
    }

#ifdef GOOD_GRAPH_TIME_LOGGING
    time_log.time_gg_optimization += timer.toc();
#endif

}

#endif

void Optimizer::LocalBundleAdjustment(KeyFrame *pKF, bool *pbStopFlag, Map *pMap,
                                      vector<size_t> &mvKeyFrameList, vector<size_t> &mvFixedFrameList,
                                      size_t &num_Point)
{
    // Local KeyFrames: First Breath Search from Current Keyframe
    list<KeyFrame *> lLocalKeyFrames;

    lLocalKeyFrames.push_back(pKF);
    pKF->mnBALocalForKF = pKF->mnId;

    const vector<KeyFrame *> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
    for (int i = 0, iend = vNeighKFs.size(); i < iend; i++)
    {
        KeyFrame *pKFi = vNeighKFs[i];
        pKFi->mnBALocalForKF = pKF->mnId;
        if (!pKFi->isBad())
        {
            lLocalKeyFrames.push_back(pKFi);
            //
            //            if (lLocalKeyFrames.size() >= 20)
            //                break ;
        }
    }

    // Local MapPoints seen in Local KeyFrames
    list<MapPoint *> lLocalMapPoints;
    for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++)
    {
        vector<MapPoint *> vpMPs = (*lit)->GetMapPointMatches();
        for (vector<MapPoint *>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; vit++)
        {
            MapPoint *pMP = *vit;
            if (pMP)
                if (!pMP->isBad())
                    if (pMP->mnBALocalForKF != pKF->mnId)
                    {
                        lLocalMapPoints.push_back(pMP);
                        pMP->mnBALocalForKF = pKF->mnId;
                    }
        }
    }
    //
    num_Point = lLocalMapPoints.size();

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    list<KeyFrame *> lFixedCameras;
    for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++)
    {
        map<KeyFrame *, size_t> observations = (*lit)->GetObservations();
        for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
        {
            KeyFrame *pKFi = mit->first;

            if (pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId)
            {
                pKFi->mnBAFixedForKF = pKF->mnId;
                if (!pKFi->isBad())
                {
                    lFixedCameras.push_back(pKFi);

                    //
                    //                    if (lLocalKeyFrames.size() + lFixedCameras.size() >= 50) {
                    //                        //
                    //                        lLocalMapPoints.erase(lit, lend);
                    //                        break ;
                    //                    }
                }
            }
        }
    }


#ifdef ENABLE_MAP_IO
    // DEBUG
    //    cout << "Before: num of local KF = " << lLocalKeyFrames.size()
    //	 << "; num of fixed KF = " << lFixedCameras.size();
    // move loaded KF from lLocalKeyFrames to lFixedCameras
    list<KeyFrame*> lTmpKeyFrames;
    // vector<list<KeyFrame*>::iterator> vEraseIndex;
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        if(!pKFi->isBad()) {
            if (pKFi->mbFixedKF)
                lFixedCameras.push_back(pKFi);
            else
                lTmpKeyFrames.push_back(pKFi);
        }
    }
    //
    lLocalKeyFrames = lTmpKeyFrames;
    lTmpKeyFrames.clear();
    // DEBUG
    //    cout << "After: num of local KF = " << lLocalKeyFrames.size()
    //	 << "; num of fixed KF = " << lFixedCameras.size();
#endif


    // Set Local KeyFrame vertices
    mvKeyFrameList.clear();
    for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++)
    {
        mvKeyFrameList.push_back((*lit)->mnId);
    }

    // Set Fixed KeyFrame vertices
    mvFixedFrameList.clear();
    for (list<KeyFrame *>::iterator lit = lFixedCameras.begin(), lend = lFixedCameras.end(); lit != lend; lit++)
    {
        mvFixedFrameList.push_back((*lit)->mnId);
    }

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if (pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    unsigned long maxKFid = 0;

    // Set Local KeyFrame vertices
    for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++)
    {
        KeyFrame *pKFi = *lit;
        g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(pKFi->mnId == 0);
        optimizer.addVertex(vSE3);
        if (pKFi->mnId > maxKFid)
            maxKFid = pKFi->mnId;
    }

    // Set Fixed KeyFrame vertices
    for (list<KeyFrame *>::iterator lit = lFixedCameras.begin(), lend = lFixedCameras.end(); lit != lend; lit++)
    {
        KeyFrame *pKFi = *lit;
        g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(true);
        optimizer.addVertex(vSE3);
        if (pKFi->mnId > maxKFid)
            maxKFid = pKFi->mnId;
    }

    // Set MapPoint vertices
    const int nExpectedSize = (lLocalKeyFrames.size()+lFixedCameras.size())*lLocalMapPoints.size();

    vector<g2o::EdgeSE3ProjectXYZ*> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
    vpEdgesStereo.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);

    const float thHuberMono = sqrt(5.991);
    const float thHuberStereo = sqrt(7.815);

    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        int id = pMP->mnId+maxKFid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        const map<KeyFrame*,size_t> observations = pMP->GetObservations();

        //Set edges
        for(map<KeyFrame*,size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if(!pKFi->isBad())
            {
                const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];

                // Monocular observation
                if(pKFi->mvuRight[mit->second]<0)
                {
                    Eigen::Matrix<double,2,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;

                    optimizer.addEdge(e);
                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);
                }
                else // Stereo observation
                {
                    Eigen::Matrix<double,3,1> obs;
                    const float kp_ur = pKFi->mvuRight[mit->second];
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                    e->setInformation(Info);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;
                    e->bf = pKFi->mbf;

                    optimizer.addEdge(e);
                    vpEdgesStereo.push_back(e);
                    vpEdgeKFStereo.push_back(pKFi);
                    vpMapPointEdgeStereo.push_back(pMP);
                }
            }
        }
    }

    if(pbStopFlag)
        if(*pbStopFlag)
            return;

    optimizer.initializeOptimization();
    optimizer.optimize(5);

    bool bDoMore= true;

    if(pbStopFlag)
        if(*pbStopFlag)
            bDoMore = false;

    if(bDoMore)
    {

        // Check inlier observations
        for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
        {
            g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
            MapPoint* pMP = vpMapPointEdgeMono[i];

            if(pMP->isBad())
                continue;

            if(e->chi2()>5.991 || !e->isDepthPositive())
            {
                e->setLevel(1);
            }

            e->setRobustKernel(0);
        }

        for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
        {
            g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
            MapPoint* pMP = vpMapPointEdgeStereo[i];

            if(pMP->isBad())
                continue;

            if(e->chi2()>7.815 || !e->isDepthPositive())
            {
                e->setLevel(1);
            }

            e->setRobustKernel(0);
        }

        // Optimize again without the outliers

        optimizer.initializeOptimization(0);
        optimizer.optimize(10);

    }

    vector<pair<KeyFrame*,MapPoint*> > vToErase;
    vToErase.reserve(vpEdgesMono.size()+vpEdgesStereo.size());

    // Check inlier observations
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
    {
        g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>5.991 || !e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKFMono[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }
    }

    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
    {
        g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
        MapPoint* pMP = vpMapPointEdgeStereo[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>7.815 || !e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKFStereo[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }
    }

    // Get Map Mutex
    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    if(!vToErase.empty())
    {
        for(size_t i=0;i<vToErase.size();i++)
        {
            KeyFrame* pKFi = vToErase[i].first;
            MapPoint* pMPi = vToErase[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }
    }

    // Recover optimized data

    //Keyframes
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKF = *lit;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        pKF->SetPose(Converter::toCvMat(SE3quat));
    }

    //Points
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
        pMP->UpdateNormalAndDepth();
    }
}


void Optimizer::OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFrame *, set<KeyFrame *> > &LoopConnections, const bool &bFixScale)
{
    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    g2o::BlockSolver_7_3::LinearSolverType * linearSolver =
            new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
    g2o::BlockSolver_7_3 * solver_ptr= new g2o::BlockSolver_7_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    solver->setUserLambdaInit(1e-16);
    optimizer.setAlgorithm(solver);

    const vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    const vector<MapPoint*> vpMPs = pMap->GetAllMapPoints();

    const unsigned int nMaxKFid = pMap->GetMaxKFid();

    vector<g2o::Sim3,Eigen::aligned_allocator<g2o::Sim3> > vScw(nMaxKFid+1);
    vector<g2o::Sim3,Eigen::aligned_allocator<g2o::Sim3> > vCorrectedSwc(nMaxKFid+1);
    vector<g2o::VertexSim3Expmap*> vpVertices(nMaxKFid+1);

    const int minFeat = 100;

    // Set KeyFrame vertices
    for(size_t i=0, iend=vpKFs.size(); i<iend;i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

        const int nIDi = pKF->mnId;

        LoopClosing::KeyFrameAndPose::const_iterator it = CorrectedSim3.find(pKF);

        if(it!=CorrectedSim3.end())
        {
            vScw[nIDi] = it->second;
            VSim3->setEstimate(it->second);
        }
        else
        {
            Eigen::Matrix<double,3,3> Rcw = Converter::toMatrix3d(pKF->GetRotation());
            Eigen::Matrix<double,3,1> tcw = Converter::toVector3d(pKF->GetTranslation());
            g2o::Sim3 Siw(Rcw,tcw,1.0);
            vScw[nIDi] = Siw;
            VSim3->setEstimate(Siw);
        }

        if(pKF==pLoopKF)
            VSim3->setFixed(true);

        VSim3->setId(nIDi);
        VSim3->setMarginalized(false);
        VSim3->_fix_scale = bFixScale;

        optimizer.addVertex(VSim3);

        vpVertices[nIDi]=VSim3;
    }

    set<pair<long unsigned int, long unsigned int>> sInsertedEdges;

    const Eigen::Matrix<double, 7, 7> matLambda = Eigen::Matrix<double, 7, 7>::Identity();

    // Set Loop edges
    for(map<KeyFrame *, set<KeyFrame *> >::const_iterator mit = LoopConnections.begin(), mend=LoopConnections.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        const long unsigned int nIDi = pKF->mnId;
        const set<KeyFrame*> &spConnections = mit->second;
        const g2o::Sim3 Siw = vScw[nIDi];
        const g2o::Sim3 Swi = Siw.inverse();

        for(set<KeyFrame*>::const_iterator sit=spConnections.begin(), send=spConnections.end(); sit!=send; sit++)
        {
            const long unsigned int nIDj = (*sit)->mnId;
            if((nIDi!=pCurKF->mnId || nIDj!=pLoopKF->mnId) && pKF->GetWeight(*sit)<minFeat)
                continue;

            const g2o::Sim3 Sjw = vScw[nIDj];
            const g2o::Sim3 Sji = Sjw * Swi;

            g2o::EdgeSim3* e = new g2o::EdgeSim3();
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
            e->setMeasurement(Sji);

            e->information() = matLambda;

            optimizer.addEdge(e);

            sInsertedEdges.insert(make_pair(min(nIDi,nIDj),max(nIDi,nIDj)));
        }
    }

    // Set normal edges
    for(size_t i=0, iend=vpKFs.size(); i<iend; i++)
    {
        KeyFrame* pKF = vpKFs[i];

        const int nIDi = pKF->mnId;

        g2o::Sim3 Swi;

        LoopClosing::KeyFrameAndPose::const_iterator iti = NonCorrectedSim3.find(pKF);

        if(iti!=NonCorrectedSim3.end())
            Swi = (iti->second).inverse();
        else
            Swi = vScw[nIDi].inverse();

        KeyFrame* pParentKF = pKF->GetParent();

        // Spanning tree edge
        if(pParentKF)
        {
            int nIDj = pParentKF->mnId;

            g2o::Sim3 Sjw;

            LoopClosing::KeyFrameAndPose::const_iterator itj = NonCorrectedSim3.find(pParentKF);

            if(itj!=NonCorrectedSim3.end())
                Sjw = itj->second;
            else
                Sjw = vScw[nIDj];

            g2o::Sim3 Sji = Sjw * Swi;

            g2o::EdgeSim3* e = new g2o::EdgeSim3();
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
            e->setMeasurement(Sji);

            e->information() = matLambda;
            optimizer.addEdge(e);
        }

        // Loop edges
        const set<KeyFrame*> sLoopEdges = pKF->GetLoopEdges();
        for(set<KeyFrame*>::const_iterator sit=sLoopEdges.begin(), send=sLoopEdges.end(); sit!=send; sit++)
        {
            KeyFrame* pLKF = *sit;
            if(pLKF->mnId<pKF->mnId)
            {
                g2o::Sim3 Slw;

                LoopClosing::KeyFrameAndPose::const_iterator itl = NonCorrectedSim3.find(pLKF);

                if(itl!=NonCorrectedSim3.end())
                    Slw = itl->second;
                else
                    Slw = vScw[pLKF->mnId];

                g2o::Sim3 Sli = Slw * Swi;
                g2o::EdgeSim3* el = new g2o::EdgeSim3();
                el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pLKF->mnId)));
                el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
                el->setMeasurement(Sli);
                el->information() = matLambda;
                optimizer.addEdge(el);
            }
        }

        // Covisibility graph edges
        const vector<KeyFrame*> vpConnectedKFs = pKF->GetCovisiblesByWeight(minFeat);
        for(vector<KeyFrame*>::const_iterator vit=vpConnectedKFs.begin(); vit!=vpConnectedKFs.end(); vit++)
        {
            KeyFrame* pKFn = *vit;
            if(pKFn && pKFn!=pParentKF && !pKF->hasChild(pKFn) && !sLoopEdges.count(pKFn))
            {
                if(!pKFn->isBad() && pKFn->mnId<pKF->mnId)
                {
                    if(sInsertedEdges.count(make_pair(min(pKF->mnId,pKFn->mnId),max(pKF->mnId,pKFn->mnId))))
                        continue;

                    g2o::Sim3 Snw;

                    LoopClosing::KeyFrameAndPose::const_iterator itn = NonCorrectedSim3.find(pKFn);

                    if(itn!=NonCorrectedSim3.end())
                        Snw = itn->second;
                    else
                        Snw = vScw[pKFn->mnId];

                    g2o::Sim3 Sni = Snw * Swi;

                    g2o::EdgeSim3* en = new g2o::EdgeSim3();
                    en->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFn->mnId)));
                    en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
                    en->setMeasurement(Sni);
                    en->information() = matLambda;
                    optimizer.addEdge(en);
                }
            }
        }
    }

    // Optimize!
    optimizer.initializeOptimization();

#ifdef GROUND_TRUTH_GEN_MODE
    optimizer.optimize(40);
#else
    optimizer.optimize(20);
#endif

    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
    for(size_t i=0;i<vpKFs.size();i++)
    {
        KeyFrame* pKFi = vpKFs[i];

        const int nIDi = pKFi->mnId;

        g2o::VertexSim3Expmap* VSim3 = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(nIDi));
        g2o::Sim3 CorrectedSiw =  VSim3->estimate();
        vCorrectedSwc[nIDi]=CorrectedSiw.inverse();
        Eigen::Matrix3d eigR = CorrectedSiw.rotation().toRotationMatrix();
        Eigen::Vector3d eigt = CorrectedSiw.translation();
        double s = CorrectedSiw.scale();

        eigt *=(1./s); //[R t/s;0 1]

        cv::Mat Tiw = Converter::toCvSE3(eigR,eigt);

        pKFi->SetPose(Tiw);
    }

    // Correct points. Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
    for(size_t i=0, iend=vpMPs.size(); i<iend; i++)
    {
        MapPoint* pMP = vpMPs[i];

        if(pMP->isBad())
            continue;

        int nIDr;
        if(pMP->mnCorrectedByKF==pCurKF->mnId)
        {
            nIDr = pMP->mnCorrectedReference;
        }
        else
        {
            KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();
            nIDr = pRefKF->mnId;
        }


        g2o::Sim3 Srw = vScw[nIDr];
        g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

        cv::Mat P3Dw = pMP->GetWorldPos();
        Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
        Eigen::Matrix<double,3,1> eigCorrectedP3Dw = correctedSwr.map(Srw.map(eigP3Dw));

        cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
        pMP->SetWorldPos(cvCorrectedP3Dw);

        pMP->UpdateNormalAndDepth();
    }
}

int Optimizer::OptimizeSim3(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches1, g2o::Sim3 &g2oS12, const float th2, const bool bFixScale)
{
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    // Calibration
    const cv::Mat &K1 = pKF1->mK;
    const cv::Mat &K2 = pKF2->mK;

    // Camera poses
    const cv::Mat R1w = pKF1->GetRotation();
    const cv::Mat t1w = pKF1->GetTranslation();
    const cv::Mat R2w = pKF2->GetRotation();
    const cv::Mat t2w = pKF2->GetTranslation();

    // Set Sim3 vertex
    g2o::VertexSim3Expmap * vSim3 = new g2o::VertexSim3Expmap();
    vSim3->_fix_scale=bFixScale;
    vSim3->setEstimate(g2oS12);
    vSim3->setId(0);
    vSim3->setFixed(false);
    vSim3->_principle_point1[0] = K1.at<float>(0,2);
    vSim3->_principle_point1[1] = K1.at<float>(1,2);
    vSim3->_focal_length1[0] = K1.at<float>(0,0);
    vSim3->_focal_length1[1] = K1.at<float>(1,1);
    vSim3->_principle_point2[0] = K2.at<float>(0,2);
    vSim3->_principle_point2[1] = K2.at<float>(1,2);
    vSim3->_focal_length2[0] = K2.at<float>(0,0);
    vSim3->_focal_length2[1] = K2.at<float>(1,1);
    optimizer.addVertex(vSim3);

    // Set MapPoint vertices
    const int N = vpMatches1.size();
    const vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
    vector<g2o::EdgeSim3ProjectXYZ*> vpEdges12;
    vector<g2o::EdgeInverseSim3ProjectXYZ*> vpEdges21;
    vector<size_t> vnIndexEdge;

    vnIndexEdge.reserve(2*N);
    vpEdges12.reserve(2*N);
    vpEdges21.reserve(2*N);

    const float deltaHuber = sqrt(th2);

    int nCorrespondences = 0;

    for(int i=0; i<N; i++)
    {
        if(!vpMatches1[i])
            continue;

        MapPoint* pMP1 = vpMapPoints1[i];
        MapPoint* pMP2 = vpMatches1[i];

        const int id1 = 2*i+1;
        const int id2 = 2*(i+1);

        const int i2 = pMP2->GetIndexInKeyFrame(pKF2);

        if(pMP1 && pMP2)
        {
            if(!pMP1->isBad() && !pMP2->isBad() && i2>=0)
            {
                g2o::VertexSBAPointXYZ* vPoint1 = new g2o::VertexSBAPointXYZ();
                cv::Mat P3D1w = pMP1->GetWorldPos();
                cv::Mat P3D1c = R1w*P3D1w + t1w;
                vPoint1->setEstimate(Converter::toVector3d(P3D1c));
                vPoint1->setId(id1);
                vPoint1->setFixed(true);
                optimizer.addVertex(vPoint1);

                g2o::VertexSBAPointXYZ* vPoint2 = new g2o::VertexSBAPointXYZ();
                cv::Mat P3D2w = pMP2->GetWorldPos();
                cv::Mat P3D2c = R2w*P3D2w + t2w;
                vPoint2->setEstimate(Converter::toVector3d(P3D2c));
                vPoint2->setId(id2);
                vPoint2->setFixed(true);
                optimizer.addVertex(vPoint2);
            }
            else
                continue;
        }
        else
            continue;

        nCorrespondences++;

        // Set edge x1 = S12*X2
        Eigen::Matrix<double,2,1> obs1;
        const cv::KeyPoint &kpUn1 = pKF1->mvKeysUn[i];
        obs1 << kpUn1.pt.x, kpUn1.pt.y;

        g2o::EdgeSim3ProjectXYZ* e12 = new g2o::EdgeSim3ProjectXYZ();
        e12->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id2)));
        e12->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        e12->setMeasurement(obs1);
        const float &invSigmaSquare1 = pKF1->mvInvLevelSigma2[kpUn1.octave];
        e12->setInformation(Eigen::Matrix2d::Identity()*invSigmaSquare1);

        g2o::RobustKernelHuber* rk1 = new g2o::RobustKernelHuber;
        e12->setRobustKernel(rk1);
        rk1->setDelta(deltaHuber);
        optimizer.addEdge(e12);

        // Set edge x2 = S21*X1
        Eigen::Matrix<double,2,1> obs2;
        const cv::KeyPoint &kpUn2 = pKF2->mvKeysUn[i2];
        obs2 << kpUn2.pt.x, kpUn2.pt.y;

        g2o::EdgeInverseSim3ProjectXYZ* e21 = new g2o::EdgeInverseSim3ProjectXYZ();

        e21->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id1)));
        e21->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        e21->setMeasurement(obs2);
        float invSigmaSquare2 = pKF2->mvInvLevelSigma2[kpUn2.octave];
        e21->setInformation(Eigen::Matrix2d::Identity()*invSigmaSquare2);

        g2o::RobustKernelHuber* rk2 = new g2o::RobustKernelHuber;
        e21->setRobustKernel(rk2);
        rk2->setDelta(deltaHuber);
        optimizer.addEdge(e21);

        vpEdges12.push_back(e12);
        vpEdges21.push_back(e21);
        vnIndexEdge.push_back(i);
    }

    // Optimize!
    optimizer.initializeOptimization();

#ifdef GROUND_TRUTH_GEN_MODE
    optimizer.optimize(15);
#else
    optimizer.optimize(5);
#endif

    // Check inliers
    int nBad=0;
    for(size_t i=0; i<vpEdges12.size();i++)
    {
        g2o::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
        g2o::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
        if(!e12 || !e21)
            continue;

        if(e12->chi2()>th2 || e21->chi2()>th2)
        {
            size_t idx = vnIndexEdge[i];
            vpMatches1[idx]=static_cast<MapPoint*>(NULL);
            optimizer.removeEdge(e12);
            optimizer.removeEdge(e21);
            vpEdges12[i]=static_cast<g2o::EdgeSim3ProjectXYZ*>(NULL);
            vpEdges21[i]=static_cast<g2o::EdgeInverseSim3ProjectXYZ*>(NULL);
            nBad++;
        }
    }

    int nMoreIterations;
    if(nBad>0)
        nMoreIterations=10;
    else
        nMoreIterations=5;

    if(nCorrespondences-nBad<10)
        return 0;

    // Optimize again only with inliers

    optimizer.initializeOptimization();

#ifdef GROUND_TRUTH_GEN_MODE
    optimizer.optimize(nMoreIterations * 2);
#else
    optimizer.optimize(nMoreIterations);
#endif

    int nIn = 0;
    for(size_t i=0; i<vpEdges12.size();i++)
    {
        g2o::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
        g2o::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
        if(!e12 || !e21)
            continue;

        if(e12->chi2()>th2 || e21->chi2()>th2)
        {
            size_t idx = vnIndexEdge[i];
            vpMatches1[idx]=static_cast<MapPoint*>(NULL);
        }
        else
            nIn++;
    }

    // Recover optimized Sim3
    g2o::VertexSim3Expmap* vSim3_recov = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(0));
    g2oS12= vSim3_recov->estimate();

    return nIn;
}


} //namespace ORB_SLAM2
