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


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/core.hpp>
#include "../../../../include/System.h"
#include "../../../../include/MapPublisher.h"

#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>

//
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/publisher.h>

#include <sparse_stereo_msgs/TrackedPoint.h>
#include <sparse_stereo_msgs/TrackedPointList.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define PUB_TRACKED_FEATURES

// #define MAP_PUBLISH


using namespace std;

#define FRAME_WITH_INFO_PUBLISH

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM), tf_listener_(tf_buffer_){
#ifdef MAP_PUBLISH
      mnMapRefreshCounter = 0;
#endif
    }

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    void GrabOdom(const nav_msgs::Odometry::ConstPtr& msg);

    void GrabPath(const nav_msgs::Path::ConstPtr    & msg);

    ORB_SLAM2::System* mpSLAM;

    double timeStamp;  
    cv::Mat Tmat;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    ros::Publisher mpCameraPosePublisher, mpCameraPoseInIMUPublisher;

    ros::Publisher trackedFeaturesPublisher;

    geometry_msgs::TransformStamped cam_base_static_trans_, cam_odom_init_trans_, cam_imu_static_trans_;
    tf2::Transform cam_odom_init_tf2_, cam_base_static_tf2_, cam_imu_static_tf2_;
    bool cam_base_trans_set_ = false;
    
#ifdef MAP_PUBLISH
    size_t mnMapRefreshCounter;
    ORB_SLAM2::MapPublisher* mpMapPub;
#endif

#ifdef FRAME_WITH_INFO_PUBLISH
    ros::Publisher mpFrameWithInfoPublisher;
#endif
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 9)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings budget_per_frame "
             << " do_viz "
             << " topic_image_rgb topic_image_depth path_to_traj path_to_map" << endl;
        ros::shutdown();
        return 1;
    }    

    bool do_viz;
    stringstream s1(argv[4]);
    s1 >> boolalpha >> do_viz;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,do_viz);

    SLAM.SetConstrPerFrame(std::atoi(argv[3]));

#ifdef LOGGING_KF_LIST
    std::string fNameRealTimeBA = std::string(argv[8]) + "_Log_BA.txt";
    std::cout << std::endl << "Saving BA Log to Log_BA.txt" << std::endl;
#endif

#ifdef REALTIME_TRAJ_LOGGING
    std::string fNameRealTimeTrack = std::string(argv[7]) + "_AllFrameTrajectory.txt";
    std::cout << std::endl << "Saving AllFrame Trajectory to AllFrameTrajectory.txt" << std::endl;
    SLAM.SetRealTimeFileStream(fNameRealTimeTrack);
#endif
    
#ifdef ENABLE_MAP_IO
    SLAM.LoadMap(std::string(argv[8]));
    SLAM.ForceRelocTracker();
#endif
    
    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;

//    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
//    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "camera/depth_registered/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, argv[5], 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, argv[6], 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    //
    ros::Subscriber sub2 = nh.subscribe("/odom", 100, &ImageGrabber::GrabOdom, &igb);
    // ros::Subscriber sub2 = nh.subscribe("/desired_path", 100, &ImageGrabber::GrabPath, &igb);


    // TODO
        // figure out the proper queue size
    igb.mpCameraPosePublisher = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("ORB_SLAM/camera_pose", 100);
    igb.mpCameraPoseInIMUPublisher = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("ORB_SLAM/camera_pose_in_imu", 100);

#ifdef PUB_TRACKED_FEATURES
    igb.trackedFeaturesPublisher = nh.advertise<sparse_stereo_msgs::TrackedPointList>("ORB_SLAM/tracked_features", 100);
#endif

#ifdef FRAME_WITH_INFO_PUBLISH
    igb.mpFrameWithInfoPublisher = nh.advertise<sensor_msgs::Image>("ORB_SLAM/frame_with_info", 100);
#endif

#ifdef MAP_PUBLISH
    igb.mpMapPub = new ORB_SLAM2::MapPublisher(SLAM.mpMap);
    ROS_INFO_STREAM("Initialized map publisher");
#endif

while(ros::ok()) 
    ros::spin();
    // ros::spin();

    cout << "ros_rgbd: done with spin!" << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM( std::string(argv[7]) + "_KeyFrameTrajectory.txt" );
    SLAM.SaveTrackingLog( std::string(argv[7]) + "_Log.txt" );
#ifdef LOCAL_BA_TIME_LOGGING
    SLAM.SaveMappingLog( std::string(argv[7]) + "_Log_Mapping.txt" );
#endif

/*
#ifdef ENABLE_MAP_IO
   
   SLAM.SaveMap(std::string(argv[8]));
   // SLAM.SaveMap(std::string(argv[7]) + "_Map/");
#endif
*/

    std::cout << "Finished saving!" << std::endl;

    ros::shutdown();

    cout << "ros_rgbd: done with ros Shutdown!" << endl;

    // Stop all threads
    SLAM.Shutdown();
    cout << "ros_rgbd: done with SLAM Shutdown!" << endl;

    return 0;
}

void ImageGrabber::GrabOdom(const nav_msgs::Odometry::ConstPtr& msg) {
  /*
  ROS_INFO("Seq: [%d]", msg->header.seq);
  ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
  ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
  */
  
  // TODO
  timeStamp = msg->header.stamp.toSec();
  
  mpSLAM->mpTracker->BufferingOdom(
                timeStamp, 
                msg->pose.pose.position.x, 
                msg->pose.pose.position.y, 
                msg->pose.pose.position.z, 
                msg->pose.pose.orientation.w, 
                msg->pose.pose.orientation.x, 
                msg->pose.pose.orientation.y, 
                msg->pose.pose.orientation.z
  );
}

void ImageGrabber::GrabPath(const nav_msgs::Path::ConstPtr& msg) {
    /*
    ROS_INFO("Seq: [%d]", msg->header.seq);
    ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
    ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
    */

    //    // densify the path
    //    // create a cubic spline interpolator
    //    nav_msgs::Path path_dense;
    //    // pointsPerUnit, skipPoints, useEndConditions, useMiddleConditions);
    //    path_smoothing::CubicSplineInterpolator csi(double(100.0),
    //                                                (unsigned int)0,
    //                                                true,
    //                                                true);
    //    csi.interpolatePath(*msg, path_dense);

    size_t N = msg->poses.size();
    //    ROS_INFO("Size of path: before [%d] vs. after [%d]", msg->poses.size(), N);
    for (size_t i=0; i<N; ++i) {

        timeStamp = msg->poses[i].header.stamp.toSec();
        mpSLAM->mpTracker->BufferingOdom(
                    timeStamp,
                    msg->poses[i].pose.position.x,
                    msg->poses[i].pose.position.y,
                    msg->poses[i].pose.position.z,
                    msg->poses[i].pose.orientation.w,
                    msg->poses[i].pose.orientation.x,
                    msg->poses[i].pose.orientation.y,
                    msg->poses[i].pose.orientation.z
                    );
    }

    //    mpDensePathPub.publish(path_dense);
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD)
{

    double latency_trans = ros::Time::now().toSec() - msgRGB->header.stamp.toSec();
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat pose = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());

    if (pose.empty())
        return;


    double latency_total = ros::Time::now().toSec() - cv_ptrRGB->header.stamp.toSec();
    // ROS_INFO("ORB-SLAM Tracking Latency: %.03f sec", ros::Time::now().toSec() - cv_ptrLeft->header.stamp.toSec());
    // ROS_INFO("Image Transmision Latency: %.03f sec; Total Tracking Latency: %.03f sec", latency_trans, latency_total);
    ROS_INFO("Pose Tracking Latency: %.03f sec", latency_total - latency_trans);

    // Obtain the tracked points from RGBD
    std::vector<cv::KeyPoint> stereo_pt_l;
    std::vector<float> var_pt_l;
    std::vector<float> stereo_pt_depth;
    std::vector<long unsigned int> stereo_pt_id;

    sparse_stereo_msgs::TrackedPointList tracked_points;
    tracked_points.header = msgRGB->header;

    // Use tracked map points and keypoints
    std::vector<ORB_SLAM2::MapPoint *> stereo_map_pts = mpSLAM->GetTrackedMapPoints();
    stereo_pt_l = mpSLAM->GetTrackedKeyPointsUn();
    var_pt_l = mpSLAM->mpTracker->mCurrentFrame.mvLevelSigma2;

    tracked_points.tracked_list.clear();
    for(size_t i = 0; i < stereo_map_pts.size(); i++)
    {
        ORB_SLAM2::MapPoint* map_pt = stereo_map_pts[i];
        if(map_pt != NULL && !map_pt->isBad())
        {
            cv::Mat Pw = map_pt->GetWorldPos(), Pc;
            if (mpSLAM->mpTracker->mCurrentFrame.WorldToCameraPoint(Pw, Pc) == true && Pc.at<float>(2) < 100) 
            {
                sparse_stereo_msgs::TrackedPoint tracked_pt;
                tracked_pt.header = msgRGB->header;

                tracked_pt.depth = Pc.at<float>(2);
                tracked_pt.id = map_pt->mnId;
                tracked_pt.u_l = stereo_pt_l[i].pt.x;
                tracked_pt.v_l = stereo_pt_l[i].pt.y;

                tracked_pt.var = var_pt_l[stereo_pt_l[i].octave];

                tracked_pt.pt_camera.x = Pc.at<float>(0);
                tracked_pt.pt_camera.y = Pc.at<float>(1);
                tracked_pt.pt_camera.z = Pc.at<float>(2);
                
                tracked_points.tracked_list.push_back(tracked_pt);
            }
        }
    }

    trackedFeaturesPublisher.publish(tracked_points);

    ROS_INFO_STREAM("Tracked [ " << tracked_points.tracked_list.size() <<" ] features.");

    /*
    // global left handed coordinate system 
    static cv::Mat pose_prev = cv::Mat::eye(4,4, CV_32F);
    static cv::Mat world_lh = cv::Mat::eye(4,4, CV_32F);
    // matrix to flip signs of sinus in rotation matrix, not sure why we need to do that
    static const cv::Mat flipSign = (cv::Mat_<float>(4,4) <<   1,-1,-1, 1,
                                                               -1, 1,-1, 1,
                                                               -1,-1, 1, 1,
                                                                1, 1, 1, 1);

    //prev_pose * T = pose
    cv::Mat translation =  (pose * pose_prev.inv()).mul(flipSign);
    world_lh = world_lh * translation;
    pose_prev = pose.clone();


    // transform into global right handed coordinate system, publish in ROS
    tf::Matrix3x3 cameraRotation_rh(  - world_lh.at<float>(0,0),   world_lh.at<float>(0,1),   world_lh.at<float>(0,2),
                                  - world_lh.at<float>(1,0),   world_lh.at<float>(1,1),   world_lh.at<float>(1,2),
                                    world_lh.at<float>(2,0), - world_lh.at<float>(2,1), - world_lh.at<float>(2,2));

    tf::Vector3 cameraTranslation_rh( world_lh.at<float>(0,3),world_lh.at<float>(1,3), - world_lh.at<float>(2,3) );

    //rotate 270deg about x and 270deg about x to get ENU: x forward, y left, z up
    const tf::Matrix3x3 rotation270degXZ(   0, 1, 0,
                                            0, 0, 1,
                                            1, 0, 0);
    
    static tf::TransformBroadcaster br;

    tf::Matrix3x3 globalRotation_rh = cameraRotation_rh * rotation270degXZ;
    tf::Vector3 globalTranslation_rh = cameraTranslation_rh * rotation270degXZ;
    tf::Transform transform = tf::Transform(globalRotation_rh, globalTranslation_rh);
    br.sendTransform(tf::StampedTransform(transform, cv_ptrRGB->header.stamp, "map", "camera_pose"));
*/
    
/// broadcast campose pose message
    // camera pose
 /*
         tf::Matrix3x3 R( 0,  0,  1,
                        -1,  0,  0,
                         0, -1,  0);
         tf::Transform T ( R * tf::Matrix3x3( transform.getRotation() ), R * transform.getOrigin() );
  */
 
    cv::Mat Rwc = pose.rowRange(0,3).colRange(0,3).t();
    cv::Mat twc = -Rwc*pose.rowRange(0,3).col(3);
    tf2::Matrix3x3 M(Rwc.at<float>(0,0),Rwc.at<float>(0,1),Rwc.at<float>(0,2),
                    Rwc.at<float>(1,0),Rwc.at<float>(1,1),Rwc.at<float>(1,2),
                    Rwc.at<float>(2,0),Rwc.at<float>(2,1),Rwc.at<float>(2,2));
    tf2::Vector3 V(twc.at<float>(0), twc.at<float>(1), twc.at<float>(2));

    tf2::Transform tfTcw(M,V);
	geometry_msgs::Transform gmTwc;
    tf2::convert(tfTcw, gmTwc);
        
    geometry_msgs::Pose camera_pose;
    camera_pose.position.x = gmTwc.translation.x;
    camera_pose.position.y = gmTwc.translation.y;
    camera_pose.position.z = gmTwc.translation.z;
    camera_pose.orientation = gmTwc.rotation;

    if(!cam_base_trans_set_)
    {
        cam_odom_init_trans_ = tf_buffer_.lookupTransform("odom", cv_ptrRGB->header.frame_id, ros::Time(0));
        cam_base_static_trans_ = tf_buffer_.lookupTransform("base_footprint", cv_ptrRGB->header.frame_id, ros::Time(0));
        cam_imu_static_trans_ = tf_buffer_.lookupTransform("gyro_link", cv_ptrRGB->header.frame_id, ros::Time(0));

        tf2::Vector3 cam_odom_init_t(cam_odom_init_trans_.transform.translation.x, cam_odom_init_trans_.transform.translation.y, cam_odom_init_trans_.transform.translation.z);
        tf2::Quaternion cam_odom_init_q(cam_odom_init_trans_.transform.rotation.x, cam_odom_init_trans_.transform.rotation.y, cam_odom_init_trans_.transform.rotation.z, cam_odom_init_trans_.transform.rotation.w);
        cam_odom_init_tf2_.setOrigin(cam_odom_init_t);
        cam_odom_init_tf2_.setRotation(cam_odom_init_q);

        tf2::Vector3 cam_base_static_t(cam_base_static_trans_.transform.translation.x, cam_base_static_trans_.transform.translation.y, cam_base_static_trans_.transform.translation.z);
        tf2::Quaternion cam_base_static_q(cam_base_static_trans_.transform.rotation.x, cam_base_static_trans_.transform.rotation.y, cam_base_static_trans_.transform.rotation.z, cam_base_static_trans_.transform.rotation.w);
        cam_base_static_tf2_.setOrigin(cam_base_static_t);
        cam_base_static_tf2_.setRotation(cam_base_static_q);

        tf2::Vector3 cam_imu_static_t(cam_imu_static_trans_.transform.translation.x, cam_imu_static_trans_.transform.translation.y, cam_imu_static_trans_.transform.translation.z);
        tf2::Quaternion cam_imu_static_q(cam_imu_static_trans_.transform.rotation.x, cam_imu_static_trans_.transform.rotation.y, cam_imu_static_trans_.transform.rotation.z, cam_imu_static_trans_.transform.rotation.w);
        cam_imu_static_tf2_.setOrigin(cam_imu_static_t);
        cam_imu_static_tf2_.setRotation(cam_imu_static_q);

        cam_base_trans_set_ = true;
    }

    tf2::Transform camera_in_odom = cam_odom_init_tf2_ * tfTcw * cam_base_static_tf2_.inverse();
    camera_pose.position.x = camera_in_odom.getOrigin().getX();
    camera_pose.position.y = camera_in_odom.getOrigin().getY();
    camera_pose.position.z = camera_in_odom.getOrigin().getZ();
    camera_pose.orientation.x = camera_in_odom.getRotation().getX();
    camera_pose.orientation.y = camera_in_odom.getRotation().getY();
    camera_pose.orientation.z = camera_in_odom.getRotation().getZ();
    camera_pose.orientation.w = camera_in_odom.getRotation().getW();
	
    // This is only for visualization purpose now.
	geometry_msgs::PoseWithCovarianceStamped camera_odom;
    camera_odom.header.frame_id = "odom";
    camera_odom.header.stamp = cv_ptrRGB->header.stamp;
    camera_odom.pose.pose = camera_pose;

    mpCameraPosePublisher.publish(camera_odom);

//
// by default, an additional transform is applied to make camera pose and body frame aligned
// which is assumed in msf
#ifdef INIT_WITH_ARUCHO
	tf2::Matrix3x3 Ric(   0, -1, 0,
                             0, 0, -1,
                             1, 0, 0);
/*	tf::Matrix3x3 Ric(   0, 0, 1,
                             -1, 0, 0,
                             0, -1, 0);*/
        tf2::Transform tfTiw ( tf::Matrix3x3( tfTcw.getRotation() ) * Ric, tfTcw.getOrigin() );
#else
	tf2::Matrix3x3 Ric( 0,  0,  1,
                         -1,  0,  0,
                         0,  -1,  0);
       tf2::Transform tfTiw ( Ric * tf2::Matrix3x3( tfTcw.getRotation() ), Ric * tfTcw.getOrigin() );
#endif

	geometry_msgs::Transform gmTwi;
    tf2::convert(tfTiw, gmTwi);
        
    geometry_msgs::Pose camera_pose_in_imu;
    camera_pose_in_imu.position.x = gmTwi.translation.x;
    camera_pose_in_imu.position.y = gmTwi.translation.y;
    camera_pose_in_imu.position.z = gmTwi.translation.z;
    camera_pose_in_imu.orientation = gmTwi.rotation;
	
	geometry_msgs::PoseWithCovarianceStamped camera_odom_in_imu;
    camera_odom_in_imu.header.frame_id = "map";
    camera_odom_in_imu.header.stamp = cv_ptrRGB->header.stamp;
    camera_odom_in_imu.pose.pose = camera_pose_in_imu;

    mpCameraPoseInIMUPublisher.publish(camera_odom_in_imu);

	
	/*
   geometry_msgs::Transform gmTwc;
   tf::transformTFToMsg(transform, gmTwc);
        
    geometry_msgs::Pose camera_pose;
        camera_pose.position.x = gmTwc.translation.x;
        camera_pose.position.y = gmTwc.translation.y;
        camera_pose.position.z = gmTwc.translation.z;
        camera_pose.orientation = gmTwc.rotation;
        
    geometry_msgs::PoseWithCovarianceStamped camera_odom;
     camera_odom.header.frame_id = "map";
        camera_odom.header.stamp = cv_ptrRGB->header.stamp;
        camera_odom.pose.pose = camera_pose;
    
      mpCameraPosePublisher.publish(camera_odom);
*/

#ifdef FRAME_WITH_INFO_PUBLISH
    if (mpSLAM != NULL && mpSLAM->mpFrameDrawer != NULL) {
        cv::Mat fr_info_cv = mpSLAM->mpFrameDrawer->DrawFrame();
        cv_bridge::CvImage out_msg;
        out_msg.header   = cv_ptrRGB->header; // Same timestamp and tf frame as input image
        out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
        out_msg.image    = fr_info_cv; // Your cv::Mat
        mpFrameWithInfoPublisher.publish(out_msg.toImageMsg());
    }
#endif

#ifdef MAP_PUBLISH
    if (mnMapRefreshCounter % 30 == 1) {
    	// publish map points
    	mpMapPub->Refresh();
    }
    mnMapRefreshCounter ++;
#endif
}


