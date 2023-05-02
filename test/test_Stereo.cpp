// Copyright 2005, Google Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following disclaimer
// in the documentation and/or other materials provided with the
// distribution.
//     * Neither the name of Google Inc. nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// A sample program demonstrating using Google C++ testing framework.
//
// Author: wan@google.com (Zhanyong Wan)


// In this example, we use a more advanced feature of Google Test called
// test fixture.
//
// A test fixture is a place to hold objects and functions shared by
// all tests in a test case.  Using a test fixture avoids duplicating
// the test code necessary to initialize and cleanup those common
// objects for each test.  It is also useful for defining sub-routines
// that your tests need to invoke a lot.
//
// <TechnicalDetails>
//
// The tests share the test fixture in the sense of code sharing, not
// data sharing.  Each test is given its own fresh copy of the
// fixture.  You cannot expect the data modified by one test to be
// passed on to another test, which is a bad idea.
//
// The reason for this design is that tests should be independent and
// repeatable.  In particular, a test should not fail as the result of
// another test's failure.  If one test depends on info produced by
// another test, then the two tests should really be one big test.
//
// The macros for indicating the success/failure of a test
// (EXPECT_TRUE, FAIL, etc) need to know what the current test is
// (when Google Test prints the test result, it tells you which test
// each failure belongs to).  Technically, these macros invoke a
// member function of the Test class.  Therefore, you cannot use them
// in a global function.  That's why you should put test sub-routines
// in a test fixture.
//
// </TechnicalDetails>

#include "Observability.h"
#include <opencv2/highgui/highgui.hpp>
#include "gtest/gtest.h"

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs/legacy/constants_c.h>
#include <opencv2/core/eigen.hpp>

using namespace ORB_SLAM2;

#define USE_FISHEYE_DISTORTION

namespace {
// To use a test fixture, derive a class from testing::Test.
class TestStereo : public testing::Test {
protected:  // You should make the members protected s.t. they can be
    // accessed from sub-classes.

    // virtual void SetUp() will be called before each test is run.  You
    // should define it if you need to initialize the variables.
    // Otherwise, this can be skipped.
    virtual void SetUp() {
        // initialize the obs class
        //        std::cout << "start setting up" << std::endl;

        string strVocFile = "../Vocabulary/ORBvoc.bin";
//        //
//        string strSettingPath = "../Examples/Stereo/EuRoC.yaml";
//        string leftimg_filename = "../test/EuRoC_l.png";
//        string rightimg_filename = "../test/EuRoC_r.png";
        //
        string strSettingPath = "../Examples/Stereo/TUM_VI_fov.yaml";
        string leftimg_filename = "../test/TUM_l.png";
        string rightimg_filename = "../test/TUM_r.png";

        //Load ORB Vocabulary
        cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

        ORBVocabulary * mpVocabulary = new ORBVocabulary();
        bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
        if(!bVocLoad)
        {
            cerr << "Wrong path to vocabulary. " << endl;
            cerr << "Falied to open at: " << strVocFile << endl;
            exit(-1);
        }
        cout << "Vocabulary loaded!" << endl << endl;

        int nFeatures = 1000;
        double fScaleFactor = 1.2;
        int nLevels = 8;
        int fIniThFAST = 20;
        int fMinThFAST = 7;

        ORBextractor * mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
        ORBextractor * mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
        cout << "Created ORB extractor!" << endl << endl;

        //
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        float fx = fSettings["Camera.fx"];
        float fy = fSettings["Camera.fy"];
        float cx = fSettings["Camera.cx"];
        float cy = fSettings["Camera.cy"];

        cv::Mat K = cv::Mat::eye(3,3,CV_32F);
        K.at<float>(0,0) = fx;
        K.at<float>(1,1) = fy;
        K.at<float>(0,2) = cx;
        K.at<float>(1,2) = cy;

        cv::Mat DistCoef(4,1,CV_32F);
#ifdef USE_FISHEYE_DISTORTION
        DistCoef.at<float>(0) = fSettings["Camera.k1"];
        DistCoef.at<float>(1) = fSettings["Camera.k2"];
        DistCoef.at<float>(2) = fSettings["Camera.k3"];
        DistCoef.at<float>(3) = fSettings["Camera.k4"];
#else
        DistCoef.at<float>(0) = fSettings["Camera.k1"];
        DistCoef.at<float>(1) = fSettings["Camera.k2"];
        DistCoef.at<float>(2) = fSettings["Camera.p1"];
        DistCoef.at<float>(3) = fSettings["Camera.p2"];
        const float k3 = fSettings["Camera.k3"];
        if(k3!=0)
        {
            DistCoef.resize(5);
            DistCoef.at<float>(4) = k3;
        }
#endif

        double bf = fSettings["Camera.bf"];
        double ThDepth = fSettings["ThDepth"];

        image_l_ = cv::imread(leftimg_filename, CV_LOAD_IMAGE_GRAYSCALE);
        if( image_l_.empty() )
        {
            std::cout << "Unable to load " << leftimg_filename;
            return ;
        }

        image_r_ = cv::imread(rightimg_filename, CV_LOAD_IMAGE_GRAYSCALE);
        if( image_r_.empty() )
        {
            std::cout << "Unable to load " << rightimg_filename;
            return ;
        }


        // Load settings related to stereo calibration
        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fSettings["LEFT.K"] >> K_l;
        fSettings["RIGHT.K"] >> K_r;

        fSettings["LEFT.P"] >> P_l;
        fSettings["RIGHT.P"] >> P_r;

        fSettings["LEFT.R"] >> R_l;
        fSettings["RIGHT.R"] >> R_r;

        fSettings["LEFT.D"] >> D_l;
        fSettings["RIGHT.D"] >> D_r;

        int rows_l = fSettings["LEFT.height"];
        int cols_l = fSettings["LEFT.width"];
        int rows_r = fSettings["RIGHT.height"];
        int cols_r = fSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return ;
        }

        cv::Mat M1l, M2l, M1r, M2r;

#ifdef USE_FISHEYE_DISTORTION
        cv::fisheye::initUndistortRectifyMap(K_l,D_l,R_l,P_l,cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
        cv::fisheye::initUndistortRectifyMap(K_r,D_r,R_r,P_r,cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);
        cout << "finish creating equidistant rectification map!" << endl;
#else
        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);
        cout << "finish creating rad-tan rectification map!" << endl;
#endif

        cv::remap(image_l_,image_l_,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(image_r_,image_r_,M1r,M2r,cv::INTER_LINEAR);

        Eigen::MatrixXf image_lEigen, image_rEigen;
        Eigen::VectorXf distCoefEigen;
        Eigen::Matrix3f KEigen;

        cv::cv2eigen(image_l_, image_lEigen);
        cv::cv2eigen(image_r_, image_rEigen);
        cv::cv2eigen(DistCoef, distCoefEigen);
        cv::cv2eigen(K, KEigen);


        pFrame_ = new Frame(image_lEigen,image_rEigen,99999999,mpORBextractorLeft,mpORBextractorRight,mpVocabulary,KEigen,distCoefEigen,bf,ThDepth);
    }

    // virtual void TearDown() will be called after each test is run.
    // You should define it if there is cleanup work to do.  Otherwise,
    // you don't have to provide it.
    //
    // virtual void TearDown() {
    // }

    // Declares the variables your tests want to use.
    //    Observability * obs_;
    Frame * pFrame_;
    cv::Mat image_l_;
    cv::Mat image_r_;
};


TEST_F(TestStereo, Detection) {
    //
    //    std::cout << "subset selection from " << pFrame_->mvpMapPoints.size() << " lmks:" << std::endl;
    cv::Mat image_detect;
    pFrame_->plotStereoDetection(image_l_, image_r_, image_detect);

    std::string save_path = "./stereo_detection_tum_2.png";
    cv::imwrite( save_path, image_detect );

    //
    cv::Mat image_match;
    pFrame_->plotStereoMatching(image_l_, image_r_, image_match);

    save_path = "./stereo_matching_tum_2.png";
    cv::imwrite( save_path, image_match );
}

}  // namespace
