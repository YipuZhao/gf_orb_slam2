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

#include "KeyFrame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include <mutex>

#include <opencv2/core/eigen.hpp>


namespace ORB_SLAM2
{

long unsigned int KeyFrame::nNextId = 0;

#ifdef ENABLE_ANTICIPATION_IN_GRAPH
KeyFrame::KeyFrame(const double &timeStamp, const cv::Mat &Tcw,
                   const float &fx_, const float &fy_, const float &cx_, const float &cy_, const float &mb_) :
    mTimeStamp(timeStamp), fx(fx_), fy(fy_), cx(cx_), cy(cy_), mb(mb_) {
    // no need to assign mnId = nNextId++;
    SetPose(Tcw);
}
#endif

KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB) : mnFrameId(F.mnId), mTimeStamp(F.mTimeStamp), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
    mfGridElementWidthInv(F.mfGridElementWidthInv), mfGridElementHeightInv(F.mfGridElementHeightInv),
    mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
    mnBALocalForKFCand(0), mnBAFixedForKFCand(0), mnBALocalCount(0),
    mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0),
    fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), invfx(F.invfx), invfy(F.invfy),
    mbf(F.mbf), mb(F.mb), mThDepth(F.mThDepth), N(F.N), mvKeys(F.mvKeys), mvKeysUn(F.mvKeysUn),
    mvuRight(F.mvuRight), mvDepth(F.mvDepth), mDescriptors(F.getDescriptorCV()),
    mBowVec(F.mBowVec), mFeatVec(F.mFeatVec), mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor),
    mfLogScaleFactor(F.mfLogScaleFactor), mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
    mvInvLevelSigma2(F.mvInvLevelSigma2), mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX),
    mnMaxY(F.mnMaxY), mK(F.getmkCV()), mvpMapPoints(F.mvpMapPoints), mpKeyFrameDB(pKFDB),
    mpORBvocabulary(F.mpORBvocabulary), mbFirstConnection(true), mpParent(NULL), mbNotErase(false),
    mbToBeErased(false), mbBad(false), mHalfBaseline(F.mb/2), mpMap(pMap)
{
    mnId=nNextId++;

    mGrid.resize(mnGridCols);
    for(int i=0; i<mnGridCols;i++)
    {
        mGrid[i].resize(mnGridRows);
        for(int j=0; j<mnGridRows; j++)
            mGrid[i][j] = F.mGrid[i][j];
    }

    SetPose(F.mTcw);
    
    //
    mbFixedKF = false;

    mNumVisibleMpt = 0;
}


#ifdef ENABLE_MAP_IO
//
KeyFrame::KeyFrame(cv::FileStorage & fs, Map * mMap,
                   ORBVocabulary* mVocabulary, KeyFrameDatabase* mKeyFrameDatabase) {

    double val_tmp;

    fs["nNextId"] >> val_tmp; nNextId = (unsigned long)val_tmp;
    fs["mnId"] >> val_tmp; mnId = (unsigned long)val_tmp;
    fs["mnFrameId"] >> val_tmp; mnFrameId = (const unsigned long)val_tmp;

    fs["mTimeStamp"] >> mTimeStamp;

    fs["mnGridCols"] >> mnGridCols;
    fs["mnGridRows"] >> mnGridRows;
    fs["mfGridElementWidthInv"] >> mfGridElementWidthInv;
    fs["mfGridElementHeightInv"] >> mfGridElementHeightInv;

    fs["mnTrackReferenceForFrame"] >> val_tmp; mnTrackReferenceForFrame = (unsigned long)val_tmp;
    fs["mnFuseTargetForKF"] >> val_tmp; mnFuseTargetForKF = (unsigned long)val_tmp;
    fs["mnBALocalForKF"] >> val_tmp; mnBALocalForKF = (unsigned long)val_tmp;
    fs["mnBAFixedForKF"] >> val_tmp; mnBAFixedForKF = (unsigned long)val_tmp;

    fs["mnLoopQuery"] >> val_tmp; mnLoopQuery = (unsigned long)val_tmp;
    fs["mnLoopWords"] >> mnLoopWords;
    fs["mLoopScore"] >> mLoopScore;

    //    fs["mnRelocQuery"] >> val_tmp; mnRelocQuery = (unsigned long)val_tmp;
    //    fs["mnRelocWords"] >> mnRelocWords;
    //    fs["mRelocScore"] >> mRelocScore;
    mnRelocQuery = ULONG_MAX;
    mnRelocWords = 0;
    mRelocScore = 0;

    fs["fx"] >> fx;
    fs["fy"] >> fy;
    fs["cx"] >> cx;
    fs["cy"] >> cy;

    //    fs["Tcw"] >> Tcw;
    cv::Mat Tcw_;
    fs["Tcw"] >> Tcw_;
    this->SetPose(Tcw_);
    fs["Ow"] >> Ow;

    //    fs["im"] >> im;
    fs["mnMinX"] >> mnMinX;
    fs["mnMinY"] >> mnMinY;
    fs["mnMaxX"] >> mnMaxX;
    fs["mnMaxY"] >> mnMaxY;
    fs["mK"] >> mK;

    cv::FileNode mvKeysFileNode = fs["mvKeys"];//>> no operator overloading for KeyPoint
    read(mvKeysFileNode, mvKeys);
    cv::FileNode mvKeysUnFileNode = fs["mvKeysUn"];
    read(mvKeysUnFileNode, mvKeysUn);
    assert(mvKeysUn.size() == mvKeys.size());
    fs["mDescriptors"] >> mDescriptors;

    fs["mvuRight"] >> mvuRight;
    assert(mvuRight.size() == mvKeys.size());
    fs["mvDepth"] >> mvDepth;
    assert(mvDepth.size() == mvKeys.size());
    //    cv::FileNode mvuRightFileNode = fs["mvuRight"];
    //    read(mvuRightFileNode, mvuRight);
    //    cv::FileNode mvDepthFileNode = fs["mvDepth"];
    //    read(mvDepthFileNode, mvDepth);

    mpKeyFrameDB = mKeyFrameDatabase;
    mpORBvocabulary = mVocabulary;
    vector<cv::Mat> vCurrentDesc = ORB_SLAM2::Converter::toDescriptorVector(mDescriptors);
    mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);

    std::vector<int> mGrid_serialized;
    std::vector<int> mGridSize;
    fs["mGrid"] >> mGrid_serialized;
    fs["mGridSize"] >> mGridSize;
    for(size_t i = 0; i < FRAME_GRID_COLS; i++){
        std::vector<std::vector<size_t> > mGridyz;
        for(size_t j = 0; j < FRAME_GRID_ROWS; j++){
            std::vector<size_t> mGridz;
            int sz = mGridSize.front();
            mGridSize.erase(mGridSize.begin());
            for(int k = 0; k < sz; k++){
                mGridz.push_back(size_t(mGrid_serialized.front()));
                mGrid_serialized.erase(mGrid_serialized.begin());
            }
            mGridyz.push_back(mGridz);
        }
        mGrid.push_back(mGridyz);
    }

    fs["mvOrderedWeights"] >> mvOrderedWeights;
    fs["mbFirstConnection"] >> mbFirstConnection;

    fs["mbNotErase"] >> mbNotErase;
    fs["mbToBeErased"] >> mbToBeErased;
    fs["mbBad"] >> mbBad;

    fs["mnScaleLevels"] >> mnScaleLevels;
    fs["mvScaleFactors"] >> mvScaleFactors;
    fs["mvLevelSigma2"] >> mvLevelSigma2;
    fs["mvInvLevelSigma2"] >> mvInvLevelSigma2;

    mpParent = NULL;

    mpMap = mMap;

    mvpMapPoints.clear();
    
    //
    mbFixedKF = true;

    mNumVisibleMpt = 0;

    mnBALocalCount = 10;
}
//
#endif

void KeyFrame::ExportToYML(cv::FileStorage & fs) {

    if (!fs.isOpened())
        return ;

    write(fs, "nNextId", double(nNextId));
    write(fs, "mnId", double(mnId));
    write(fs, "mnFrameId", double(mnFrameId));

    write(fs, "mTimeStamp", mTimeStamp);

    write(fs, "mnGridCols", mnGridCols);
    write(fs, "mnGridRows", mnGridRows);
    write(fs, "mfGridElementWidthInv", mfGridElementWidthInv);
    write(fs, "mfGridElementHeightInv", mfGridElementHeightInv);

    write(fs, "mnTrackReferenceForFrame", double(mnTrackReferenceForFrame));
    write(fs, "mnFuseTargetForKF", double(mnFuseTargetForKF));

    write(fs, "mnBALocalForKF", double(mnBALocalForKF));
    write(fs, "mnBAFixedForKF", double(mnBAFixedForKF));

    write(fs, "mnLoopQuery", double(mnLoopQuery));
    write(fs, "mnLoopWords", mnLoopWords);
    write(fs, "mLoopScore", mLoopScore);
    write(fs, "mnRelocQuery", double(mnRelocQuery));
    write(fs, "mnRelocWords", mnRelocWords);
    write(fs, "mRelocScore", mRelocScore);

    write(fs, "fx", fx);
    write(fs, "fy", fy);
    write(fs, "cx", cx);
    write(fs, "cy", cy);

    cv::Mat TcwCV, OwCV;
    cv::eigen2cv(Tcw, TcwCV);
    cv::eigen2cv(Ow, OwCV);

    write(fs, "Tcw", TcwCV);
    write(fs, "Ow", OwCV);

    //    write(fs, "im", im);
    write(fs, "mnMinX", mnMinX);
    write(fs, "mnMinY", mnMinY);
    write(fs, "mnMaxX", mnMaxX);
    write(fs, "mnMaxY", mnMaxY);
    write(fs, "mK", mK);

    write(fs, "mvKeys", mvKeys);
    write(fs, "mvKeysUn", mvKeysUn);
    write(fs, "mDescriptors", mDescriptors);

    write(fs, "mvuRight", mvuRight);
    write(fs, "mvDepth", mvDepth);

    //    cout << "here 1" << endl;

    vector<double> mvMapPoints;
    for(size_t i = 0; i < mvpMapPoints.size(); i++){
        if(mvpMapPoints[i] != NULL)
            mvMapPoints.push_back(double(mvpMapPoints[i]->mnId));//might point to NULL?
        else
            mvMapPoints.push_back(-1);
    }
    write(fs, "mvMapPoints", mvMapPoints);

    //     cout << "here 2" << endl;

    std::vector<int> mGrid_serialized;
    std::vector<int> mGridSize;
    for(size_t i = 0; i < mGrid.size(); i++){
        for(size_t j = 0; j < mGrid[i].size(); j++){
            mGridSize.push_back(mGrid[i][j].size());
            if(mGrid[i][j].size() == 0)
                continue;
            else{
                for(size_t k = 0; k < mGrid[i][j].size(); k++){
                    mGrid_serialized.push_back(int(mGrid[i][j][k]));
                }
            }
        }
    }
    write(fs, "mGrid", mGrid_serialized);
    write(fs, "mGridSize", mGridSize);

    //     cout << "here 3" << endl;
    /*
        for(size_t i = 0; i < pKF->mGrid.size(); i++){
            for(size_t j = 0; j < pKF->mGrid[i].size(); j++){
                f<<endl<<i<<" "<<j<<endl;
                for(size_t k = 0; k < pKF->mGrid[i][j].size(); k++){
                    f<<pKF->mGrid[i][j][k];
                }
            }
        }
        */
    vector<double> mConnectedKeyFrameWeights_first;
    vector<int> mConnectedKeyFrameWeights_second;
    std::map<ORB_SLAM2::KeyFrame*,int>::iterator iteckfw = mConnectedKeyFrameWeights.begin();
    for(; iteckfw != mConnectedKeyFrameWeights.end(); iteckfw++){
        mConnectedKeyFrameWeights_first.push_back(double(iteckfw->first->mnId));
        mConnectedKeyFrameWeights_second.push_back(iteckfw->second);
    }
    write(fs, "mConnectedKeyFrameWeights_first", mConnectedKeyFrameWeights_first);
    write(fs, "mConnectedKeyFrameWeights_second", mConnectedKeyFrameWeights_second);

    //     cout << "here 4" << endl;

    vector<double> mvOrderedConnectedKeyFrames;
    for(size_t i = 0; i < mvpOrderedConnectedKeyFrames.size(); i++){
        mvOrderedConnectedKeyFrames.push_back(double(mvpOrderedConnectedKeyFrames[i]->mnId));
    }
    write(fs, "mvOrderedConnectedKeyFrames", mvOrderedConnectedKeyFrames);

    write(fs, "mvOrderedWeights", mvOrderedWeights);

    write(fs, "mbFirstConnection", mbFirstConnection);
    write(fs, "mParent", (mpParent == NULL)? -1:double(mpParent->mnId));

    //     cout << "here 5" << endl;

    vector<double> msChildrens;
    std::set<ORB_SLAM2::KeyFrame*>::iterator itec = mspChildrens.begin();
    for(; itec != mspChildrens.end(); itec++){
        msChildrens.push_back(double((*itec)->mnId));
    }
    write(fs, "msChildrens", msChildrens);

    vector<double> msLoopEdges;
    std::set<ORB_SLAM2::KeyFrame*>::iterator itele = mspLoopEdges.begin();
    for(; itele != mspLoopEdges.end(); itele++){
        msLoopEdges.push_back(double((*itele)->mnId));
    }
    write(fs, "msLoopEdges", msLoopEdges);

    //     cout << "here 6" << endl;

    write(fs, "mbNotErase", mbNotErase);
    write(fs, "mbToBeErased", mbToBeErased);
    write(fs, "mbBad", mbBad);

    write(fs, "mnScaleLevels", mnScaleLevels);
    write(fs, "mvScaleFactors", mvScaleFactors);
    write(fs, "mvLevelSigma2", mvLevelSigma2);
    write(fs, "mvInvLevelSigma2", mvInvLevelSigma2);

    //     cout << "here 7" << endl;
}

void KeyFrame::ComputeBoW()
{
    if(mBowVec.empty() || mFeatVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        // Feature vector associate features with nodes in the 4th level (from leaves up)
        // We assume the vocabulary tree has 6 levels, change the 4 otherwise
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}

void KeyFrame::SetPose(const Eigen::MatrixXf &Tcw_)
{
    unique_lock<mutex> lock(mMutexPose);
    Tcw = Tcw_;
    Eigen::MatrixXf Rcw = Tcw.block<3,3>(0,0);
    Eigen::VectorXf tcw = Tcw.block<3,1>(0,3);
    Eigen::MatrixXf Rwc = Rcw.transpose();
    Ow = -Rwc*tcw;

    Twc = Eigen::Matrix4f::Identity();;
    Tcw.block<3,3>(0,0) = Rwc;
    Tcw.block<3,1>(0,3) = Ow;

    Eigen::Matrix<float, 4 ,1> center;
    center << mHalfBaseline;
    center << 0;
    center << 0;
    center << 1;
    Cw = Twc*center;
}

void KeyFrame::SetPoseCV(const cv::Mat &Tcw_)
{
  Eigen::MatrixXf tcwEigen;
  cv::cv2eigen(Tcw_, tcwEigen);
    SetPose(tcwEigen);
}

Eigen::MatrixXf KeyFrame::GetPose()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw;
}

cv::Mat KeyFrame::GetPoseCV()
{
    unique_lock<mutex> lock(mMutexPose);
    cv::Mat TcwCV;
    cv::eigen2cv(Tcw, TcwCV);
    return TcwCV;
}

Eigen::MatrixXf KeyFrame::GetPoseInverse()
{
    unique_lock<mutex> lock(mMutexPose);
    return Twc;
}

cv::Mat KeyFrame::GetPoseInverseCV()
{
    unique_lock<mutex> lock(mMutexPose);
    cv::Mat TwcCV;
    cv::eigen2cv(Twc, TwcCV);
    return TwcCV.clone();
}

Eigen::MatrixXf KeyFrame::GetCameraCenter()
{
    unique_lock<mutex> lock(mMutexPose);
    return Ow;
}

cv::Mat KeyFrame::GetCameraCenterCV()
{
    unique_lock<mutex> lock(mMutexPose);
    cv::Mat OwCV;
    cv::eigen2cv(Ow, OwCV);
    return OwCV;
}

Eigen::MatrixXf KeyFrame::GetStereoCenter()
{
    unique_lock<mutex> lock(mMutexPose);
    return Cw;
}

cv::Mat KeyFrame::GetStereoCenterCV()
{
    unique_lock<mutex> lock(mMutexPose);
    cv::Mat CwCV;
    cv::eigen2cv(Cw, CwCV);
    return CwCV;
}


Eigen::MatrixXf KeyFrame::GetRotation()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.block<3,3>(0,0);
}

cv::Mat KeyFrame::GetRotationCV()
{
    unique_lock<mutex> lock(mMutexPose);
    cv::Mat TcwCV;
    Eigen::MatrixXf temp = Tcw.block<3,3>(0,0);
    cv::eigen2cv(temp, TcwCV);
    return TcwCV.clone();
}

cv::Mat KeyFrame::GetTranslationCV()
{
    unique_lock<mutex> lock(mMutexPose);
    cv::Mat TcwCV;
    Eigen::MatrixXf temp = Tcw.block<3,1>(0,3);
    cv::eigen2cv(temp, TcwCV);
    return TcwCV.clone();
}

Eigen::VectorXf KeyFrame::GetTranslation()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.block<3,1>(0,3);
}

void KeyFrame::AddConnection(KeyFrame *pKF, const int &weight)
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(!mConnectedKeyFrameWeights.count(pKF))
            mConnectedKeyFrameWeights[pKF]=weight;
        else if(mConnectedKeyFrameWeights[pKF]!=weight)
            mConnectedKeyFrameWeights[pKF]=weight;
        else
            return;
    }

    UpdateBestCovisibles();
}

void KeyFrame::UpdateBestCovisibles()
{
    unique_lock<mutex> lock(mMutexConnections);
    vector<pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(mConnectedKeyFrameWeights.size());
    for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
        vPairs.push_back(make_pair(mit->second,mit->first));

    sort(vPairs.begin(),vPairs.end());
    list<KeyFrame*> lKFs;
    list<int> lWs;
    for(size_t i=0, iend=vPairs.size(); i<iend;i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
    mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());
}

set<KeyFrame*> KeyFrame::GetConnectedKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    set<KeyFrame*> s;
    for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin();mit!=mConnectedKeyFrameWeights.end();mit++)
        s.insert(mit->first);
    return s;
}

vector<KeyFrame*> KeyFrame::GetVectorCovisibleKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    return mvpOrderedConnectedKeyFrames;
}

vector<KeyFrame*> KeyFrame::GetBestCovisibilityKeyFrames(const int &N)
{
    unique_lock<mutex> lock(mMutexConnections);
    if((int)mvpOrderedConnectedKeyFrames.size()<N)
        return mvpOrderedConnectedKeyFrames;
    else
        return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(),mvpOrderedConnectedKeyFrames.begin()+N);

}

vector<KeyFrame*> KeyFrame::GetCovisiblesByWeight(const int &w)
{
    unique_lock<mutex> lock(mMutexConnections);

    if(mvpOrderedConnectedKeyFrames.empty())
        return vector<KeyFrame*>();

    vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(),mvOrderedWeights.end(),w,KeyFrame::weightComp);
    if(it==mvOrderedWeights.end())
        return vector<KeyFrame*>();
    else
    {
        int n = it-mvOrderedWeights.begin();
        return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin()+n);
    }
}

void KeyFrame::AddCovisibleKeyFrames(KeyFrame* pKF)
{
    unique_lock<mutex> lock(mMutexConnections);
    mvpOrderedConnectedKeyFrames.push_back(pKF);
}

int KeyFrame::GetWeight(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexConnections);
    if(mConnectedKeyFrameWeights.count(pKF))
        return mConnectedKeyFrameWeights[pKF];
    else
        return 0;
}

Eigen::MatrixXf KeyFrame::GetOw() {
    return Ow;
}

void KeyFrame::AppendMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mvpMapPoints.push_back(pMP);
}

void KeyFrame::AddMapPoint(MapPoint *pMP, const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mvpMapPoints[idx]=pMP;
}

void KeyFrame::EraseMapPointMatch(const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mvpMapPoints[idx]=static_cast<MapPoint*>(NULL);
}

void KeyFrame::EraseMapPointMatch(MapPoint* pMP)
{
    int idx = pMP->GetIndexInKeyFrame(this);
    if(idx>=0)
        mvpMapPoints[idx]=static_cast<MapPoint*>(NULL);
}


void KeyFrame::ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP)
{
    mvpMapPoints[idx]=pMP;
}

set<MapPoint*> KeyFrame::GetMapPoints()
{
    unique_lock<mutex> lock(mMutexFeatures);
    set<MapPoint*> s;
    for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
    {
        if(!mvpMapPoints[i])
            continue;
        MapPoint* pMP = mvpMapPoints[i];
        if(!pMP->isBad())
            s.insert(pMP);
    }
    return s;
}

int KeyFrame::TrackedMapPoints(const int &minObs)
{
    unique_lock<mutex> lock(mMutexFeatures);

    //    cout << "N = " << N << endl;

    int nPoints=0;
    const bool bCheckObs = minObs>0;
    for(int i=0; i<N; i++)
    {
        //        cout << i << " ";
        MapPoint* pMP = mvpMapPoints[i];
        //        cout << pMP << endl;
        if(pMP)
        {
            if(!pMP->isBad())
            {
                if(bCheckObs)
                {
                    if(mvpMapPoints[i]->Observations()>=minObs)
                        nPoints++;
                }
                else
                    nPoints++;
            }
        }
    }

    return nPoints;
}

vector<MapPoint*> KeyFrame::GetMapPointMatches()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints;
}

MapPoint* KeyFrame::GetMapPoint(const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints[idx];
}


size_t KeyFrame::GetMatchNum() {
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints.size();
}


void KeyFrame::UpdateConnections()
{
    map<KeyFrame*,int> KFcounter;

    vector<MapPoint*> vpMP;

    {
        unique_lock<mutex> lockMPs(mMutexFeatures);
        vpMP = mvpMapPoints;
    }

    //For all map points in keyframe check in which other keyframes are they seen
    //Increase counter for those keyframes
    for(vector<MapPoint*>::iterator vit=vpMP.begin(), vend=vpMP.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;

        if(!pMP)
            continue;

        if(pMP->isBad())
            continue;

        map<KeyFrame*,size_t> observations = pMP->GetObservations();

        for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            if(mit->first->mnId==mnId)
                continue;
            KFcounter[mit->first]++;
        }
    }

    // This should not happen
    if(KFcounter.empty())
        return;

    //If the counter is greater than threshold add connection
    //In case no keyframe counter is over threshold add the one with maximum counter
    int nmax=0;
    KeyFrame* pKFmax=NULL;
    int th = 15;

    vector<pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(KFcounter.size());
    for(map<KeyFrame*,int>::iterator mit=KFcounter.begin(), mend=KFcounter.end(); mit!=mend; mit++)
    {
        if(mit->second>nmax)
        {
            nmax=mit->second;
            pKFmax=mit->first;
        }
        if(mit->second>=th)
        {
            vPairs.push_back(make_pair(mit->second,mit->first));
            (mit->first)->AddConnection(this,mit->second);
        }
    }

    if(vPairs.empty())
    {
        vPairs.push_back(make_pair(nmax,pKFmax));
        pKFmax->AddConnection(this,nmax);
    }

    sort(vPairs.begin(),vPairs.end());
    list<KeyFrame*> lKFs;
    list<int> lWs;
    for(size_t i=0; i<vPairs.size();i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    {
        unique_lock<mutex> lockCon(mMutexConnections);

        // mspConnectedKeyFrames = spConnectedKeyFrames;
        mConnectedKeyFrameWeights = KFcounter;
        mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
        mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());

        if(mbFirstConnection && mnId!=0)
        {
            mpParent = mvpOrderedConnectedKeyFrames.front();
            mpParent->AddChild(this);
            mbFirstConnection = false;
        }

    }
}

void KeyFrame::AddChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.insert(pKF);
}

void KeyFrame::EraseChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.erase(pKF);
}

void KeyFrame::ChangeParent(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mpParent = pKF;
    pKF->AddChild(this);
    //    cout << "ChangeParent called!" << endl;
}

set<KeyFrame*> KeyFrame::GetChilds()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens;
}

KeyFrame* KeyFrame::GetParent()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mpParent;
}

bool KeyFrame::hasChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens.count(pKF);
}

void KeyFrame::AddLoopEdge(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mbNotErase = true;
    mspLoopEdges.insert(pKF);
}

set<KeyFrame*> KeyFrame::GetLoopEdges()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspLoopEdges;
}

void KeyFrame::SetNotErase()
{
    unique_lock<mutex> lock(mMutexConnections);
    mbNotErase = true;
}

void KeyFrame::SetErase()
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mspLoopEdges.empty())
        {
            mbNotErase = false;
        }
    }

    if(mbToBeErased)
    {
        SetBadFlag();
    }
}

void KeyFrame::SetBadFlag()
{   
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mnId==0)
            return;
        else if(mbNotErase)
        {
            mbToBeErased = true;
            return;
        }
    }

    for(map<KeyFrame*,int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
        mit->first->EraseConnection(this);

    for(size_t i=0; i<mvpMapPoints.size(); i++)
        if(mvpMapPoints[i])
            mvpMapPoints[i]->EraseObservation(this);
    {
        unique_lock<mutex> lock(mMutexConnections);
        unique_lock<mutex> lock1(mMutexFeatures);

        mConnectedKeyFrameWeights.clear();
        mvpOrderedConnectedKeyFrames.clear();

        // Update Spanning Tree
        set<KeyFrame*> sParentCandidates;
        sParentCandidates.insert(mpParent);

        // Assign at each iteration one children with a parent (the pair with highest covisibility weight)
        // Include that children as new parent candidate for the rest
        while(!mspChildrens.empty())
        {
            bool bContinue = false;

            int max = -1;
            KeyFrame* pC;
            KeyFrame* pP;

            for(set<KeyFrame*>::iterator sit=mspChildrens.begin(), send=mspChildrens.end(); sit!=send; sit++)
            {
                KeyFrame* pKF = *sit;
                if(pKF->isBad())
                    continue;

                // Check if a parent candidate is connected to the keyframe
                vector<KeyFrame*> vpConnected = pKF->GetVectorCovisibleKeyFrames();
                for(size_t i=0, iend=vpConnected.size(); i<iend; i++)
                {
                    for(set<KeyFrame*>::iterator spcit=sParentCandidates.begin(), spcend=sParentCandidates.end(); spcit!=spcend; spcit++)
                    {
                        if(vpConnected[i]->mnId == (*spcit)->mnId)
                        {
                            int w = pKF->GetWeight(vpConnected[i]);
                            if(w>max)
                            {
                                pC = pKF;
                                pP = vpConnected[i];
                                max = w;
                                bContinue = true;
                            }
                        }
                    }
                }
            }

            if(bContinue)
            {
                pC->ChangeParent(pP);
                sParentCandidates.insert(pC);
                mspChildrens.erase(pC);
            }
            else
                break;
        }

        // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
        if(!mspChildrens.empty())
            for(set<KeyFrame*>::iterator sit=mspChildrens.begin(); sit!=mspChildrens.end(); sit++)
            {
                (*sit)->ChangeParent(mpParent);
            }

        mpParent->EraseChild(this);
        mTcp = Tcw * mpParent->GetPoseInverse();
        mbBad = true;
    }


    mpMap->EraseKeyFrame(this);
    mpKeyFrameDB->erase(this);
}

bool KeyFrame::isBad()
{
    unique_lock<mutex> lock(mMutexConnections);
    return mbBad;
}

void KeyFrame::EraseConnection(KeyFrame* pKF)
{
    bool bUpdate = false;
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mConnectedKeyFrameWeights.count(pKF))
        {
            mConnectedKeyFrameWeights.erase(pKF);
            bUpdate=true;
        }
    }

    if(bUpdate)
        UpdateBestCovisibles();
}

vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=mnGridCols)
        return vIndices;

    const int nMaxCellX = min((int)mnGridCols-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=mnGridRows)
        return vIndices;

    const int nMaxCellY = min((int)mnGridRows-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mGrid[ix][iy];
            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool KeyFrame::IsInImage(const float &x, const float &y) const
{
    return (x>=mnMinX && x<mnMaxX && y>=mnMinY && y<mnMaxY);
}

Eigen::MatrixXf KeyFrame::UnprojectStereo(int i)
{
    const float z = mvDepth[i];
    if(z>0)
    {
        const float u = mvKeys[i].pt.x;
        const float v = mvKeys[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;

        Eigen::Matrix<float, 3, 1> x3Dc;
        x3Dc << x,y,z;

        unique_lock<mutex> lock(mMutexPose);
        return Twc.block<3,3>(0,0) * x3Dc + Twc.block<3,1>(0,3);
    }
    else
        return Eigen::MatrixXf();
}

float KeyFrame::ComputeSceneMedianDepth(const int q)
{
    vector<MapPoint*> vpMapPoints;
    Eigen::MatrixXf Tcw_;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPose);
        vpMapPoints = mvpMapPoints;
        Tcw_ = Tcw;
    }

    vector<float> vDepths;
    vDepths.reserve(N);
    Eigen::Matrix<float,1,3> Rcw2 = Tcw_.row(2);
    float zcw = Tcw_(2,3);
    for(int i=0; i<N; i++)
    {
        if(mvpMapPoints[i])
        {
            MapPoint* pMP = mvpMapPoints[i];
            Eigen::MatrixXf x3Dw = pMP->GetWorldPosEigen();
            Eigen::Map<Eigen::RowVectorXf> v1(x3Dw.data(), x3Dw.size());

            float z = Rcw2.dot(v1)+zcw;
            vDepths.push_back(z);
        }
    }

    sort(vDepths.begin(),vDepths.end());

    return vDepths[(vDepths.size()-1)/q];
}

} //namespace ORB_SLAM
