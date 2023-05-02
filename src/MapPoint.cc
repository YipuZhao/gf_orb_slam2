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

#include "MapPoint.h"
#include "ORBmatcher.h"

#include <mutex>

namespace ORB_SLAM2
{

long unsigned int MapPoint::nNextId = 0;
mutex MapPoint::mGlobalMutex;

MapPoint::MapPoint(const cv::Mat &Pos, KeyFrame *pRefKF, Map* pMap):
    mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(static_cast<MapPoint*>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap), mnIdCoVisible(0), mnIdMapHashed(0), mnIdCandidates(-1),
    mnQueriedScore(0), mnIdRelocalized(0), mnIdLoopClosure(0)
{
    Pos.copyTo(mWorldPos);
    mNormalVector = cv::Mat::zeros(3,1,CV_32F);

    // temporalObs
    ObsScore = -1.0;
    ObsRank = 0;
    //
    matchedAtFrameId = 0;
    updateAtFrameId = 0;
    goodAtFrameId = 0;
    mnUsedForLocalMap = 0;
    //
    u_proj = FLT_MAX;
    v_proj = FLT_MAX;

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId = nNextId++;

// #ifdef ONLINE_TABLE_SELECTION
    mvbActiveHashTables = std::vector<bool>(NUM_TOTAL_HASHTABLES, false);
    // mvbActiveHashTables.resize(NUM_TOTAL_HASHTABLES);
    // std::fill(mvbActiveHashTables.begin(), mvbActiveHashTables.end(), false);
// #endif
    mvbHashed = std::vector<bool>(NUM_TOTAL_HASHTABLES, false);
//    mvbQueried = std::vector<bool>(NUM_TOTAL_HASHTABLES, false);
}

MapPoint::MapPoint(const cv::Mat &Pos, Map* pMap, Frame* pFrame, const int &idxF):
    mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mnTrackReferenceForFrame(0), mnLastFrameSeen(0),
    mnBALocalForKF(0), mnFuseCandidateForKF(0),mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(static_cast<KeyFrame*>(NULL)), mnVisible(1),
    mnFound(1), mbBad(false), mpReplaced(NULL), mpMap(pMap), mnIdCoVisible(0), mnIdMapHashed(0), mnIdCandidates(-1), mnQueriedScore(0), mnIdRelocalized(0), mnIdLoopClosure(0)
{
    Pos.copyTo(mWorldPos);
    cv::Mat Ow = pFrame->GetCameraCenterCV();
    mNormalVector = mWorldPos - Ow;
    mNormalVector = mNormalVector / cv::norm(mNormalVector);

    // temporalObs
    ObsScore = -1.0;
    ObsRank = 0;
    //
    matchedAtFrameId = 0;
    updateAtFrameId = 0;
    goodAtFrameId = 0;
    mnUsedForLocalMap = 0;
    //
    u_proj = FLT_MAX;
    v_proj = FLT_MAX;

    cv::Mat PC = Pos - Ow;
    const float dist = cv::norm(PC);
    const int level = pFrame->mvKeysUn[idxF].octave;
    const float levelScaleFactor = pFrame->mvScaleFactors[level];
    const int nLevels = pFrame->mnScaleLevels;

    mfMaxDistance = dist * levelScaleFactor;
    mfMinDistance = mfMaxDistance / pFrame->mvScaleFactors[nLevels - 1];

    cv::eigen2cv(pFrame->mDescriptors, mDescriptor);
    mDescriptor = mDescriptor.row(idxF);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId = nNextId++;

// #ifdef ONLINE_TABLE_SELECTION
    mvbActiveHashTables = std::vector<bool>(NUM_TOTAL_HASHTABLES, false);
    // mvbActiveHashTables.resize(NUM_TOTAL_HASHTABLES);
    // std::fill(mvbActiveHashTables.begin(), mvbActiveHashTables.end(), false);
// #endif
    mvbHashed = std::vector<bool>(NUM_TOTAL_HASHTABLES, false);
//    mvbQueried = std::vector<bool>(NUM_TOTAL_HASHTABLES, false);
}

MapPoint::MapPoint(cv::FileStorage &fs, Map *pMap) : nObs(0), mpReplaced(static_cast<MapPoint *>(NULL))
{

    double val_tmp;

    fs["mnFirstKFid"] >> val_tmp;
    mnFirstKFid = (long)val_tmp;

    fs["mTrackProjX"] >> mTrackProjX;
    fs["mTrackProjY"] >> mTrackProjY;
    fs["mbTrackInView"] >> mbTrackInView;
    fs["mnTrackScaleLevel"] >> mnTrackScaleLevel;
    fs["mTrackViewCos"] >> mTrackViewCos;

    fs["mnTrackReferenceForFrame"] >> val_tmp;
    mnTrackReferenceForFrame = (unsigned long)val_tmp;
    fs["mnLastFrameSeen"] >> val_tmp;
    mnLastFrameSeen = (unsigned long)val_tmp;

    fs["mnBALocalForKF"] >> val_tmp;
    mnBALocalForKF = (unsigned long)val_tmp;
    fs["mnBAGlobalForKF"] >> val_tmp;
    mnBAGlobalForKF = (unsigned long)val_tmp;
    fs["mnFuseCandidateForKF"] >> val_tmp;
    mnFuseCandidateForKF = (unsigned long)val_tmp;

    fs["mnLoopPointForKF"] >> val_tmp;
    mnLoopPointForKF = (unsigned long)val_tmp;
    fs["mnCorrectedByKF"] >> val_tmp;
    mnCorrectedByKF = (unsigned long)val_tmp;
    fs["mnCorrectedReference"] >> val_tmp;
    mnCorrectedReference = (unsigned long)val_tmp;

    fs["mWorldPos"] >> mWorldPos;

    fs["mNormalVector"] >> mNormalVector;
    fs["mDescriptor"] >> mDescriptor;

    fs["mnVisible"] >> mnVisible;
    fs["mnFound"] >> mnFound;

    fs["mbBad"] >> mbBad;

    fs["mfMinDistance"] >> mfMinDistance;
    fs["mfMaxDistance"] >> mfMaxDistance;

    mpMap = pMap;

    // GF related variables
    ObsScore = -1.0;
    ObsRank = 0;
    //
    matchedAtFrameId = 0;
    updateAtFrameId = 0;
    goodAtFrameId = 0;
    mnUsedForLocalMap = 0;
    //
    u_proj = FLT_MAX;
    v_proj = FLT_MAX;

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    fs["mnId"] >> val_tmp;
    mnId = (unsigned long)val_tmp;
    fs["nNextId"] >> val_tmp;
    nNextId = (unsigned long)val_tmp;

// #ifdef ONLINE_TABLE_SELECTION
    mvbActiveHashTables = std::vector<bool>(NUM_TOTAL_HASHTABLES, false);
    // mvbActiveHashTables.resize(NUM_TOTAL_HASHTABLES);
    // std::fill(mvbActiveHashTables.begin(), mvbActiveHashTables.end(), false);
// #endif
    mvbHashed = std::vector<bool>(NUM_TOTAL_HASHTABLES, false);
//    mvbQueried = std::vector<bool>(NUM_TOTAL_HASHTABLES, false);
}

void MapPoint::ExportToYML(cv::FileStorage &fs)
{

    if (!fs.isOpened())
        return;

    unique_lock<mutex> lock(mGlobalMutex);

    write(fs, "mnId", double(mnId));
    write(fs, "nNextId", double(nNextId));
    write(fs, "mnFirstKFid", double(mnFirstKFid));

    write(fs, "mTrackProjX", mTrackProjX);
    write(fs, "mTrackProjY", mTrackProjY);
    write(fs, "mbTrackInView", mbTrackInView);
    write(fs, "mnTrackScaleLevel", mnTrackScaleLevel);
    write(fs, "mTrackViewCos", mTrackViewCos);
    write(fs, "mnTrackReferenceForFrame", double(mnTrackReferenceForFrame));
    write(fs, "mnLastFrameSeen", double(mnLastFrameSeen));

    write(fs, "mnBALocalForKF", double(mnBALocalForKF));
    write(fs, "mnBAGlobalForKF", double(mnBAGlobalForKF));
    write(fs, "mnFuseCandidateForKF", double(mnFuseCandidateForKF));

    write(fs, "mnLoopPointForKF", double(mnLoopPointForKF));
    write(fs, "mnCorrectedByKF", double(mnCorrectedByKF));
    write(fs, "mnCorrectedReference", double(mnCorrectedReference));

    write(fs, "mWorldPos", mWorldPos);

    vector<int> mObservations_first;
    vector<int> mObservations_second;
    std::map<ORB_SLAM2::KeyFrame *, size_t>::iterator iteo = mObservations.begin();
    for (; iteo != mObservations.end(); iteo++)
    {
        mObservations_first.push_back(int(iteo->first->mnId));
        mObservations_second.push_back(int(iteo->second));
    }
    write(fs, "mObservations_first", mObservations_first);
    write(fs, "mObservations_second", mObservations_second);

    write(fs, "mNormalVector", mNormalVector);
    write(fs, "mDescriptor", mDescriptor);

    write(fs, "mRefKF", double(mpRefKF->mnId));

    write(fs, "mnVisible", mnVisible);
    write(fs, "mnFound", mnFound);

    write(fs, "mbBad", mbBad);

    write(fs, "mfMinDistance", mfMinDistance);
    write(fs, "mfMaxDistance", mfMaxDistance);
}

void MapPoint::SetWorldPos(const cv::Mat &Pos)
{
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
    Pos.copyTo(mWorldPos);
}

cv::Mat MapPoint::GetWorldPos()
{
  unique_lock<mutex> lock(mMutexPos);
  return mWorldPos;
}

Eigen::MatrixXf MapPoint::GetWorldPosEigen()
{
    unique_lock<mutex> lock(mMutexPos);
    Eigen::MatrixXf mWorldPosEigen;
    cv::cv2eigen(mWorldPos, mWorldPosEigen);
    return mWorldPosEigen;
}

cv::Mat MapPoint::GetNormal()
{
    unique_lock<mutex> lock(mMutexPos);
    return mNormalVector.clone();
}

Eigen::MatrixXf MapPoint::GetNormalEigen()
{
    unique_lock<mutex> lock(mMutexPos);
    Eigen::MatrixXf mNormalVectorEigen;
    cv::cv2eigen(mNormalVector, mNormalVectorEigen);
    return mNormalVectorEigen;
}

KeyFrame* MapPoint::GetReferenceKeyFrame()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mpRefKF;
}

void MapPoint::SetReferenceKeyFrame(KeyFrame * mRKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mpRefKF = mRKF;
}

void MapPoint::AddObservation(KeyFrame* pKF, size_t idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return;
    mObservations[pKF]=idx;

    if(pKF->mvuRight[idx]>=0)
        nObs+=2;
    else
        nObs++;
}

void MapPoint::EraseObservation(KeyFrame* pKF)
{
    bool bBad=false;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mObservations.count(pKF))
        {
            int idx = mObservations[pKF];
            if(pKF->mvuRight[idx]>=0)
                nObs-=2;
            else
                nObs--;

            mObservations.erase(pKF);

            if(mpRefKF==pKF)
                mpRefKF=mObservations.begin()->first;

            // If only 2 observations or less, discard point
            if(nObs<=2)
                bBad=true;
        }
    }

    if(bBad)
        SetBadFlag();
}

map<KeyFrame*, size_t> MapPoint::GetObservations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mObservations;
}

int MapPoint::Observations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return nObs;
}

void MapPoint::SetBadFlag()
{
    map<KeyFrame*,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        mbBad=true;
        obs = mObservations;
        mObservations.clear();
    }
    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        pKF->EraseMapPointMatch(mit->second);
    }

    mpMap->EraseMapPoint(this);
}

MapPoint* MapPoint::GetReplaced()
{
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mpReplaced;
}

void MapPoint::Replace(MapPoint* pMP)
{
    if(pMP->mnId==this->mnId)
        return;

    int nvisible, nfound;
    map<KeyFrame*,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        obs=mObservations;
        mObservations.clear();
        mbBad=true;
        nvisible = mnVisible;
        nfound = mnFound;
        mpReplaced = pMP;
    }

    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        // Replace measurement in keyframe
        KeyFrame* pKF = mit->first;

        if(!pMP->IsInKeyFrame(pKF))
        {
            pKF->ReplaceMapPointMatch(mit->second, pMP);
            pMP->AddObservation(pKF,mit->second);
        }
        else
        {
            pKF->EraseMapPointMatch(mit->second);
        }
    }
    pMP->IncreaseFound(nfound);
    pMP->IncreaseVisible(nvisible);
    pMP->ComputeDistinctiveDescriptors();

    mpMap->EraseMapPoint(this);
}

bool MapPoint::isBad()
{
    unique_lock<mutex> lock(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mbBad;
}

void MapPoint::IncreaseVisible(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnVisible+=n;
}

void MapPoint::IncreaseFound(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnFound+=n;
}

float MapPoint::GetFoundRatio()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return static_cast<float>(mnFound)/mnVisible;
}

void MapPoint::ComputeDistinctiveDescriptors()
{
    // Retrieve all observed descriptors
    vector<cv::Mat> vDescriptors;

    map<KeyFrame*,size_t> observations;

    {
        unique_lock<mutex> lock1(mMutexFeatures);
        if(mbBad)
            return;
        observations=mObservations;
    }

    if(observations.empty())
        return;

    vDescriptors.reserve(observations.size());

    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;

        if(!pKF->isBad())
            vDescriptors.push_back(pKF->mDescriptors.row(mit->second));
    }

    if(vDescriptors.empty())
        return;

    // Compute distances between them
    const size_t N = vDescriptors.size();

    float Distances[N][N];
    for(size_t i=0;i<N;i++)
    {
        Distances[i][i]=0;
        for(size_t j=i+1;j<N;j++)
        {
            int distij = ORBmatcher::DescriptorDistance(vDescriptors[i],vDescriptors[j]);
            Distances[i][j]=distij;
            Distances[j][i]=distij;
        }
    }

    // Take the descriptor with least median distance to the rest
    int BestMedian = INT_MAX;
    int BestIdx = 0;
    for(size_t i=0;i<N;i++)
    {
        vector<int> vDists(Distances[i],Distances[i]+N);
        sort(vDists.begin(),vDists.end());
        int median = vDists[0.5*(N-1)];

        if(median<BestMedian)
        {
            BestMedian = median;
            BestIdx = i;
        }
    }

    {
        unique_lock<mutex> lock(mMutexFeatures);
        mDescriptor = vDescriptors[BestIdx].clone();
    }
}

cv::Mat MapPoint::GetDescriptor()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mDescriptor.clone();
}

int MapPoint::GetIndexInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return mObservations[pKF];
    else
        return -1;
}

bool MapPoint::IsInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return (mObservations.count(pKF));
}

void MapPoint::UpdateNormalAndDepth()
{
    map<KeyFrame*,size_t> observations;
    KeyFrame* pRefKF;
    cv::Mat Pos;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        if(mbBad)
            return;
        observations=mObservations;
        pRefKF = mpRefKF;
        Pos = mWorldPos.clone();
    }

    if(observations.empty())
        return;

    cv::Mat normal = cv::Mat::zeros(3,1,CV_32F);
    int n=0;
    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        cv::Mat Owi = pKF->GetCameraCenterCV();
        cv::Mat normali = mWorldPos - Owi;
        normal = normal + normali/cv::norm(normali);
        n++;
    }

//    cout << "pRefKF = " << pRefKF << endl;

    cv::Mat PC = Pos - pRefKF->GetCameraCenterCV();
    const float dist = cv::norm(PC);
    const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
    const float levelScaleFactor =  pRefKF->mvScaleFactors[level];
    const int nLevels = pRefKF->mnScaleLevels;

    {
        unique_lock<mutex> lock3(mMutexPos);
        mfMaxDistance = dist*levelScaleFactor;
        mfMinDistance = mfMaxDistance/pRefKF->mvScaleFactors[nLevels-1];
        mNormalVector = normal/n;
    }
}

float MapPoint::GetMinDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 0.8f*mfMinDistance;
}

float MapPoint::GetMaxDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 1.2f*mfMaxDistance;
}

int MapPoint::PredictScale(const float &currentDist, KeyFrame* pKF)
{
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        ratio = mfMaxDistance/currentDist;
    }

    int nScale = ceil(log(ratio)/pKF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pKF->mnScaleLevels)
        nScale = pKF->mnScaleLevels-1;

    return nScale;
}

int MapPoint::PredictScale(const float &currentDist, Frame* pF)
{
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        ratio = mfMaxDistance/currentDist;
    }

    int nScale = ceil(log(ratio)/pF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pF->mnScaleLevels)
        nScale = pF->mnScaleLevels-1;

    return nScale;
}

} // namespace ORB_SLAM2
