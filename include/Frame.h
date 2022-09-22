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

#ifndef FRAME_H
#define FRAME_H

#include <vector>

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "KeyFrame.h"
#include "ORBextractor.h"
#include "MacroDefinitions.h"

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs/legacy/constants_c.h>

// NOTE
// as an alternative of stereo pipeline, undistort the keypoints and perform stereo matching
// no image recitification is required, nor does the subpixel stereo refine is used
#define ALTER_STEREO_MATCHING
// only uncomment it for stereo pipeline
#define DELAYED_STEREO_MATCHING

// For fisheye collected sequences such as TUM VI & MYNT
// #define USE_FISHEYE_DISTORTION

// Reduction of disparity search range for map-matched points
#define DISPARITY_THRES     50.0


// For map reuse and update
// #define ENABLE_MAP_IO


// For anticipation in good graph
// #define ENABLE_ANTICIPATION_IN_GRAPH


#ifdef ENABLE_ANTICIPATION_IN_GRAPH

    // adjust the budget of local BA with anticipation info. on-the-fly
    #define ENABLE_ANTICIPATION_IN_BUDGET
    // number of historical budget for online cost estimation
    #define NUM_HISTORICAL_BUDGET   1 // 3 // 20 //

    // number of virtual KF to be included in good graph
    #define VIRTUAL_FRAME_NUM       1 // 2 //
    // number of regular frames between virtual KF
    // for EuRoC (20 fps)
    #define VIRTUAL_FRAME_STEP      10 // 5 //
    // for Gazebo (30 fps)
    // #define VIRTUAL_FRAME_STEP      15

    // #define ENABLE_PERTURBATION_TO_ODOM
    // level of gaussian noise added to ground truth anticipation
    #define ANTICIPATION_NOISE_TRAN_STD     0.0018 // 0.0035  // m per frame
    #define ANTICIPATION_NOISE_ROTA_STD     0.0008 // 0.0015  // rad per frame

#endif



namespace ORB_SLAM2
{

// NOTE
// when enabling map-reusing, make sure the grids are configured 
// indentically; changing grid configurations will lead to segfault 
// when trying to search 2D projection of 3D map points

// Optmized for 480 * 640, e.g. Hololens
//#define FRAME_GRID_COLS 48
//#define FRAME_GRID_ROWS 64

// Optmized for 640 * 480, e.g. TUM RGBD & NUIM & EuRoC
#define FRAME_GRID_COLS 64
#define FRAME_GRID_ROWS 48

// Optimzied for 512 * 512, e.g. TUM VI
//#define FRAME_GRID_COLS 64
//#define FRAME_GRID_ROWS 64

  
/* --- options of query descriptor sampling --- */
#define BUCKET_WIDTH                	50
#define BUCKET_HEIGHT               	50
#define BUCKET_MIN_FEATURES_PER_GRID    1


class MapPoint;
class KeyFrame;

struct Grid
{
    Grid()
    {
        matched_size = 0;
        kp_unmatched_indices.clear();
    }
    unsigned int matched_size;
    std::vector<unsigned int> kp_unmatched_indices;
};

class Frame
{
public:
    Frame();

    // Copy constructor.
    Frame(const Frame &frame);

    // Modified constructor for stereo cameras with stereo matching postponed
    Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp,
          ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K,
          cv::Mat &K_left, cv::Mat &distCoef_left, cv::Mat &R_left, cv::Mat &P_left,
          cv::Mat &K_right, cv::Mat &distCoef_right, cv::Mat &R_right, cv::Mat &P_right,
          const float &bf, const float &thDepth);

    // Modified constructor for lazy-stereo, as well as detection data I/O
    Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp,
          ORBextractor *extractorLeft, ORBextractor *extractorRight, ORBVocabulary *voc, cv::Mat &K,
          cv::Mat &K_left, cv::Mat &distCoef_left, cv::Mat &R_left, cv::Mat &P_left,
          cv::Mat &K_right, cv::Mat &distCoef_right, cv::Mat &R_right, cv::Mat &P_right,
          const float &bf, const float &thDepth,
          const std::vector<cv::KeyPoint> &vKeys_left,
          const std::vector<cv::KeyPoint> &vKeys_right,
          const cv::Mat &mDisc_left, const cv::Mat &mDisc_right);

    // Constructor for stereo cameras.
    Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp,
          ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K,
          cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Constructor for RGB-D cameras.
    Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp,
          ORBextractor* extractor, ORBVocabulary* voc, cv::Mat &K,
          cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Constructor for Monocular cameras.
    Frame(const cv::Mat &imGray, const double &timeStamp,
          ORBextractor* extractor, ORBVocabulary* voc, cv::Mat &K,
          cv::Mat &distCoef, const float &bf, const float &thDepth);

    ~Frame() {};

    // Extract ORB on the image. 0 for left image and 1 for right image.
    void ExtractORB(int flag, const cv::Mat &im);

    // Export detection results to YAML, which is easy to interpret and load
    void ExportToYML(cv::FileStorage &fs);

    // Compute Bag of Words representation.
    void ComputeBoW();

    // Set the camera pose.
    void SetPose(cv::Mat Tcw);


    cv::Mat getTwc();


    // return false if invalid or invisible
    inline bool WorldToCameraPoint(const cv::Mat Pw, cv::Mat & Pc){

        // 3D in camera coordinates
        Pc = mRcw*Pw+mtcw;

        //        cout << "Pc = " << Pc.at<float>(0) << ", " << Pc.at<float>(1) << ", " << Pc.at<float>(2) << endl;

        if (Pc.at<float>(2) <= 0.0f)
            return false;

        return true;
    }


    // ================================ DEBUG func ================================

    void plotStereoDetection(const cv::Mat &imLeft, const cv::Mat &imRight,
                             cv::Mat &imRes);

    void plotStereoMatching(const cv::Mat &imLeft, const cv::Mat &imRight,
                            cv::Mat &imRes);

    //

    // Computes rotation, translation and camera center matrices from the camera pose.
    void UpdatePoseMatrices();

    // Returns the camera center.
    inline cv::Mat GetCameraCenter(){
        return mOw.clone();
    }

    // Returns inverse of rotation
    inline cv::Mat GetRotationInverse(){
        return mRwc.clone();
    }

    // Check if a MapPoint is in the frustum of the camera
    // and fill variables of the MapPoint to be used by the tracking
    bool isInFrustum(MapPoint* pMP, float viewingCosLimit);

    // Compute the cell of a keypoint (return false if outside the grid)
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

    vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;

    // Search a match for each keypoint in the left image to a keypoint in the right image.
    // If there is a match, depth is computed and the right coordinate associated to the left keypoint is stored.
    void ComputeStereoMatches();

    //    void ComputeStereoMatches_Undistorted();
    inline void PrepareStereoCandidates() {

        mvuRight = vector<float>(N,-1.0f);
        mvDepth = vector<float>(N,-1.0f);

        mvStereoMatched = vector<bool>(N,false);

        const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;

        //Assign keypoints to row table
        mvRowIndices = vector<vector<size_t> >(nRows,vector<size_t>());

        for(int i=0; i<nRows; i++)
            mvRowIndices[i].reserve(200);

        const int Nr = mvKeysRightUn.size();

        for(int iR=0; iR<Nr; iR++)
        {
            const cv::KeyPoint &kp = mvKeysRightUn[iR];
            const float &kpY = kp.pt.y;

            const float r = 2.0f*mvScaleFactors[mvKeysRightUn[iR].octave];
            const int maxr = std::min(float(nRows-1), ceil(kpY+r));
            const int minr = std::max(0.0f, floor(kpY-r));

            for(int yi=minr;yi<=maxr;yi++)
                mvRowIndices[yi].push_back(iR);
        }

        // For each left keypoint search a match in the right image
        mvDistIdx.reserve(N);
        mvDistIdx.clear();
    }

    bool ComputeStereoMatch_OnePoint(const int iL, const int thOrbDist, const float minD, const float maxD);

    int ComputeStereoMatches_Undistorted(bool isOnline = false);

    int ComputeStereoMatches_Undistorted_ByBucketing(bool isOnline = false);
    static void GetUnMatchedKPbyBucketing(const Frame *pFrame, std::vector<unsigned int> &vUnMatchedKeyPoints);

    // Associate a "right" coordinate to a keypoint if there is valid depth in the depthmap.
    void ComputeStereoFromRGBD(const cv::Mat &imDepth);

    // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
    cv::Mat UnprojectStereo(const int &i);

public:
    // Vocabulary used for relocalization.
    ORBVocabulary* mpORBvocabulary;

    // Feature extractor. The right is used only in the stereo case.
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;

    // Frame timestamp.
    double mTimeStamp;

    // Calibration matrix and OpenCV distortion parameters.
    cv::Mat mK;
    static float fx;
    static float fy;
    static float cx;
    static float cy;
    static float invfx;
    static float invfy;

    // for left cam undistort
    cv::Mat mK_ori;
    cv::Mat mDistCoef;
    cv::Mat mR;
    cv::Mat mP;

    // for right cam undistort
    cv::Mat mK_right;
    cv::Mat mDistCoef_right;
    cv::Mat mR_right;
    cv::Mat mP_right;

    // Stereo baseline multiplied by fx.
    float mbf;

    // Stereo baseline in meters.
    float mb;

    // Threshold close/far points. Close points are inserted from 1 view.
    // Far points are inserted as in the monocular case from 2 views.
    float mThDepth;

    // Number of KeyPoints.
    int N;

    // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
    // In the stereo case, mvKeysUn is redundant as images must be rectified.
    // In the RGB-D case, RGB images can be distorted.
    std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
    std::vector<cv::KeyPoint> mvKeysUn;
    std::vector<cv::KeyPoint> mvKeysRightUn;

    // Corresponding stereo coordinate and depth for each keypoint.
    // "Monocular" keypoints have a negative value.
    std::vector<float> mvuRight;
    std::vector<float> mvDepth;

    //
    std::vector<bool> mvStereoMatched;
    vector<vector<size_t> > mvRowIndices;

    //
    vector<pair<int, int> > mvDistIdx;


    // Bag of Words Vector structures.
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // ORB descriptor, each row associated to a keypoint.
    cv::Mat mDescriptors, mDescriptorsRight;

    // MapPoints associated to keypoints, NULL pointer if no association.
    std::vector<MapPoint*> mvpMapPoints;
    //
    std::vector<int> mvpMatchScore;
    
    // Flag to identify outlier associations.
    std::vector<bool> mvbOutlier;

    std::vector<bool> mvbCandidate;

  //  std::vector<bool> mvbJacobBuilt;

    //
    std::vector<bool> mvbGoodFeature;

    //

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    static float mfGridElementWidthInv;
    static float mfGridElementHeightInv;
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    // Camera pose.
    cv::Mat mTcw;

    // Current and Next Frame id.
    static long unsigned int nNextId;
    long unsigned int mnId;

    // Reference Keyframe.
    KeyFrame* mpReferenceKF;

    // Scale pyramid info.
    int mnScaleLevels;
    float mfScaleFactor;
    float mfLogScaleFactor;
    vector<float> mvScaleFactors;
    vector<float> mvInvScaleFactors;
    vector<float> mvLevelSigma2;
    vector<float> mvInvLevelSigma2;

    // Undistorted Image Bounds (computed once).
    static float mnMinX;
    static float mnMaxX;
    static float mnMinY;
    static float mnMaxY;

    static bool mbInitialComputations;


private:

    // Undistort keypoints given OpenCV distortion parameters.
    // Only for the RGB-D case. Stereo must be already rectified!
    // (called in the constructor).
    void UndistortKeyPoints();

    void UndistortKeyPointsStereo();

    // Computes image bounds for the undistorted image (called in the constructor).
    void ComputeImageBounds(const cv::Mat &imLeft);

    void ComputeImageBoundsStereo(const cv::Mat &imLeft);

    // Assign keypoints to the grid for speed up feature matching (called in the constructor).
    void AssignFeaturesToGrid();

    // Rotation, translation and camera center
    cv::Mat mRcw;
    cv::Mat mtcw;
    cv::Mat mRwc;
    cv::Mat mOw; //==mtwc
};

} // namespace ORB_SLAM2

#endif // FRAME_H
