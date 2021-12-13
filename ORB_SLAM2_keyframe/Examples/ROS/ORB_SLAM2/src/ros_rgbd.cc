#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <sstream>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"System.h"
#include "Converter.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <tf2_ros/static_transform_broadcaster.h>
#include "tf2/LinearMath/Quaternion.h"
using namespace std;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class ImageGrabber
{
public:
    bool flag=false;
    nav_msgs::Path  camerapath;
    nav_msgs::Odometry odometry;
    geometry_msgs::TransformStamped odom_trans;
    tf2_ros::StaticTransformBroadcaster broadcaster;
    tf::TransformBroadcaster odombroadcaster;
    PointCloud::Ptr PointCloudMap;
    sensor_msgs::PointCloud2 GlobalMap;
    ros::NodeHandle nh;
    ros::Publisher pub_rgb,pub_depth,pub_tcw,pub_camerapath,pub_map;
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM)
    {
        //创建ros发布节点
        pub_rgb=nh.advertise<sensor_msgs::Image>("KeyFrame/RGB",10);
        pub_depth= nh.advertise<sensor_msgs::Image> ("KeyFrame/Depth", 10);
        pub_tcw= nh.advertise<nav_msgs::Odometry> ("KeyFrame/Tcw", 10);
        pub_camerapath= nh.advertise<nav_msgs::Path> ("Path",10);
        pub_map = nh.advertise<sensor_msgs::PointCloud2>("PointCloudMap",10);
    }
    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);
    ORB_SLAM2::Converter converter;
    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/color/image_raw", 15);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth/image_rect_raw", 15);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    stringstream ss;
    ss<<"/home/wanggong/catkin_ws/src/orb_slam2/ORB_SLAM2_keyframe/Examples/ROS/ORB_SLAM2/map/"<<ros::Time::now()<<".pcd";
    SLAM.SavePointCloud(ss.str());
    ros::shutdown();
    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
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
    cv::Mat Tcw;
    std::vector<float> q;
    cv::Mat t;
    bool  isKeyFrame =true;
    Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec(),isKeyFrame,true);
    mpSLAM->getPointCloudMap(PointCloudMap);
    pcl::toROSMsg(*PointCloudMap,GlobalMap);
    if (!Tcw.empty())
    {
        sensor_msgs::Image::ConstPtr rgb_msg = msgRGB;
        sensor_msgs::Image::ConstPtr depth_msg=msgD;
        Eigen::Isometry3d Tcw_ = converter.toSE3Quat(Tcw);
        Eigen::Isometry3d Twc = Eigen::Isometry3d::Identity();
        Twc = Tcw_.inverse();
        Eigen::Quaterniond Qwc = Eigen::Quaterniond(Twc.rotation());
        Eigen::Vector3d Pwc = Twc.translation();
        odom_trans.header.frame_id = "map";
        odom_trans.header.stamp = msgRGB->header.stamp;
        odom_trans.child_frame_id = "camera_link";
        odom_trans.transform.translation.x = Pwc.x();
        odom_trans.transform.translation.y = Pwc.y();
        odom_trans.transform.translation.z = Pwc.z();
        odom_trans.transform.rotation.x = Qwc.x();
        odom_trans.transform.rotation.y = Qwc.y();
        odom_trans.transform.rotation.z = Qwc.z();
        odom_trans.transform.rotation.w = Qwc.w();
        odombroadcaster.sendTransform(odom_trans);

        nav_msgs::Odometry odometry;
        odometry.header.frame_id = "map";
        odometry.header.stamp = msgRGB->header.stamp;
        odometry.pose.pose.position.x = Pwc.x();
        odometry.pose.pose.position.y = Pwc.y();
        odometry.pose.pose.position.z = Pwc.z();
        odometry.pose.pose.orientation.x = Qwc.x();
        odometry.pose.pose.orientation.y = Qwc.y();
        odometry.pose.pose.orientation.z = Qwc.z();
        odometry.pose.pose.orientation.w = Qwc.w();

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "map";
        pose_stamped.header.stamp = msgRGB->header.stamp;
        pose_stamped.pose = odometry.pose.pose;
        camerapath.header.frame_id = "map";
        camerapath.poses.push_back(pose_stamped);
        pub_camerapath.publish(camerapath);  //相机轨迹

        // GlobalMap.header.frame_id = "World";
        GlobalMap.header.frame_id = "map";
	    GlobalMap.header.stamp = msgRGB->header.stamp;
        pub_map.publish(GlobalMap);

        geometry_msgs::TransformStamped ts;
        ts.header.frame_id = "map";
        ts.header.stamp = msgRGB->header.stamp;
        ts.child_frame_id="World";
        ts.transform.translation.x=0.0;
        ts.transform.translation.y=0.0;
        ts.transform.translation.z=0.0;
        tf2::Quaternion qtn;
        qtn.setRPY(1.57,3.14,0);
        ts.transform.rotation.x = qtn.getX();
        ts.transform.rotation.y = qtn.getY();
        ts.transform.rotation.z = qtn.getZ();
        ts.transform.rotation.w = qtn.getW();
        broadcaster.sendTransform(ts);
        if(isKeyFrame)
        {
	        pub_tcw.publish(odometry);
            pub_rgb.publish(rgb_msg);
            pub_depth.publish(depth_msg);
        }
    }
}

