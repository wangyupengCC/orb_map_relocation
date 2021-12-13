#include <chrono>
#include <ctime>
#include <climits>

#include <KeyFrame.h>
#include <opencv2/highgui/highgui.hpp>
#include "Converter.h"

#include "PointCloudMapping.h"
#include <eigen3/Eigen/Geometry>
#include <thread>

namespace ORB_SLAM2
{

/*
 *
 * @ 设置点云分辨率
 */
    PointCloudMapping::PointCloudMapping(double resolution_)
    {
        resolution_ = 0.05;
        this->resolution = resolution_;
        globalMap = boost::make_shared< PointCloud >( );
        Part_tem = boost::make_shared< PointCloud >( );
        lastKeyframeSize=0;
        viewerThread = make_shared<thread>( bind(&PointCloudMapping::viewer, this ) );
    }

    PointCloudMapping::~PointCloudMapping()
    {
        viewerThread->join();
    }

// 复位点云显示模块
    void PointCloudMapping::reset()
    {
        mvPosePointClouds.clear();
        mvPointClouds.clear();
        mpointcloudID=0;
    }

    void PointCloudMapping::Save(const string& SavePoincloudPath )   //结束
    {
        string save_path = SavePoincloudPath;
        pcl::io::savePCDFile(save_path,*globalMap);
        cout<<"save pcd files to :  "<<save_path<<endl;
    }

    void PointCloudMapping::insertKeyFrame(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
    {
        unique_lock<mutex> lck(keyframeMutex);
        cv::Mat T =kf->GetPose();
        mvPosePointClouds.push_back(T.clone());    // 每个点云的pose
        colorImgs.push_back( color.clone() );
        depthImgs.push_back( depth.clone() );
        if(cx ==0 || cy ==0||fx==0||fy==0)
        {
            cx = kf->cx;
            cy = kf->cy;
            fx = kf->fx;
            fy = kf->fy;
        }
        keyFrameUpdated.notify_one();
        cout<<"receive a keyframe, id = "<<kf->mnId<<endl;
    }

    pcl::PointCloud< PointCloudMapping::PointT >::Ptr PointCloudMapping::generatePointCloud( cv::Mat& color, cv::Mat& depth)
    {
        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        PointCloud::Ptr tmp( new PointCloud() );// point cloud is null ptr
        for ( int m=0; m<depth.rows; m+=3 )
        {
            for ( int n=0; n<depth.cols; n+=3 )
            {
                float d = depth.ptr<float>(m)[n];
                if (d < 0.5 || d>6)
                    continue;
                PointT p;
                p.z = d;
                p.x = ( n - cx) * p.z / fx;
                p.y = ( m -cy) * p.z / fy;

                p.b = color.ptr<uchar>(m)[n*3];
                p.g = color.ptr<uchar>(m)[n*3+1];
                p.r = color.ptr<uchar>(m)[n*3+2];

                tmp->points.push_back(p);
            }
        }
        PointCloud::Ptr cloud_after_PassThrough(new PointCloud);
        passthrough.setInputCloud(tmp);//输入点云
        passthrough.setFilterFieldName("z");//对z轴进行操作
        passthrough.setFilterLimits(-1.0,10.0);//设置直通滤波器操作范围 KITTI 参数 -1.0,15.0
        passthrough.setFilterLimitsNegative(false);//true表示保留范围内，false表示保留范围外
        passthrough.filter(*tmp);

        passthrough.setInputCloud(tmp);//输入点云
        passthrough.setFilterFieldName("y");//对y轴进行操作
        passthrough.setFilterLimits(-6.0,6.0);//设置直通滤波器操作范围 KITTI 参数 -6.0,6.0
        passthrough.setFilterLimitsNegative(false);//true表示保留范围内，false表示保留范围外
        passthrough.filter(*tmp);

        passthrough.setInputCloud(tmp);//输入点云
        passthrough.setFilterFieldName("x");//对x轴进行操作
        passthrough.setFilterLimits(-6.0,6.0);//设置直通滤波器操作范围 KITTI 参数 -6.0,6.0
        passthrough.setFilterLimitsNegative(false);//false就是 删除此区间外的
        passthrough.filter(*cloud_after_PassThrough);
        // std::cout << "直通滤波后点云数据点数：" << cloud_after_PassThrough->points.size() << std::endl;

        PointCloud::Ptr cloud_voxel_tem(new PointCloud);
        voxel.setInputCloud(cloud_after_PassThrough);
        voxel.setLeafSize(resolution,resolution,resolution);
        voxel.filter(*cloud_voxel_tem);
        // chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        // chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
        // cout<<"********************"<<endl;
        // cout<<"generate point cloud from  kf-ID:"<<", size="<<cloud_voxel_tem->points.size()<<endl;
        // cout<<" cost time: "<<time_used.count()*1000<<" ms ."<<endl;
        // cout<<"********************"<<endl;
        return cloud_voxel_tem;
    }

/*
 *
 * 原来版本的显示函数
 * 由于随着尺寸的增加以后,显示函数会异常退出
 */
    void PointCloudMapping::viewer()
    {

        PointCloud::Ptr tem_cloud1(new PointCloud);
        PointCloud::Ptr tem_cloud2(new PointCloud);
        PointCloud::Ptr tem_cloud3(new PointCloud);
    //    pcl::visualization::CloudViewer viewer("viewer");
        while(1)
        {
            {
                unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );
                keyFrameUpdated.wait( lck_keyframeUpdated );
            }
            size_t N=0,i=0;
            {
                unique_lock<mutex> lck( keyframeMutex );
                N =mvPosePointClouds.size();
            }
            for (  i=lastKeyframeSize; i<N; i++ )
            {
                if((mvPosePointClouds.size() != colorImgs.size() )|| (mvPosePointClouds.size()!= depthImgs.size() ) || (depthImgs.size() != colorImgs.size() ))
                {
                    cout<<" depthImgs.size != colorImgs.size()  "<<endl;
                    globalMap = boost::make_shared< PointCloud >( );
                    Part_tem = boost::make_shared< PointCloud >( );
                    lastKeyframeSize=0;
                    viewerThread = make_shared<thread>( bind(&PointCloudMapping::viewer, this ) );
                    continue;
                }
                tem_cloud1 = generatePointCloud(colorImgs[i], depthImgs[i]); //生成一幅点云大约在０．１s左右
                Eigen::Isometry3d Tcw = Converter::toSE3Quat(mvPosePointClouds[i]);
                pcl::transformPointCloud( *tem_cloud1, *tem_cloud2, Tcw.inverse().matrix());
                est.setInputSource(tem_cloud2);
                est.setInputTarget(globalMap);
                pcl::Correspondences all_correspondence;
                est.determineReciprocalCorrespondences(all_correspondence,0.4);
                if (1.0*all_correspondence.size()/tem_cloud2->width<0.5)
                {
                    *globalMap += *tem_cloud2;
                }
            // viewer.showCloud( globalMap );
            cout<<"show global map, size="<<globalMap->points.size()<<endl;
            }
            int buff_length=150;
		    if(i > (buff_length))
		    {
                mvPosePointClouds.erase(mvPosePointClouds.begin(),mvPosePointClouds.begin()+buff_length);
		        depthImgs.erase(depthImgs.begin(),depthImgs.begin()+buff_length);
		        colorImgs.erase(colorImgs.begin(),colorImgs.begin()+buff_length);
		        i=i-buff_length;
		        cout<<"delete keyframe ...."<<endl;
		    }
		    lastKeyframeSize = i;
        }
    }

    void PointCloudMapping::getGlobalCloudMap(pcl::PointCloud<pcl::PointXYZRGBA> ::Ptr &outputMap)
    {
        unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );
        outputMap= globalMap;
    }
}