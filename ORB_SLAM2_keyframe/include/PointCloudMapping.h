#ifndef POINTCLOUDMAPPING_H
#define POINTCLOUDMAPPING_H

#include "System.h"
#include <condition_variable>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>


#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/features/normal_3d.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/compression/compression_profiles.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/correspondence_estimation.h>

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/core/sparse_optimizer.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM2
{

    class PointCloudMapping
    {
    public:
        typedef pcl::PointXYZRGBA PointT;
        typedef pcl::PointCloud<PointT> PointCloud;
        PointCloudMapping( double resolution_ );
        ~PointCloudMapping();
        void insertKeyFrame( KeyFrame* kf, cv::Mat& color, cv::Mat& depth );        // 插入一个keyframe，会更新一次地图
        void Save(const string& SavePoincloudPath );    //结束
        void viewer();  //点云可视化
        void getGlobalCloudMap(pcl::PointCloud<pcl::PointXYZRGBA> ::Ptr &outputMap);    //发布的点云
        void reset();
         PointCloud::Ptr globalMap;

    protected:
        double resolution;
        pcl::StatisticalOutlierRemoval<PointT> statFliter;      //点云缩减采样
        pcl::VoxelGrid<PointT>  voxel; //显示精度
        pcl::PassThrough<PointT> passthrough;
        pcl::registration::CorrespondenceEstimation<PointT,PointT> est;
        PointCloud::Ptr generatePointCloud(cv::Mat& color, cv::Mat& depth);

        PointCloud::Ptr Part_tem;//局部地图,用于大尺度场景下的重建
        shared_ptr<thread>  viewerThread;
        bool    shutDownFlag    =false; //
        mutex   shutDownMutex;

        bool    dataupdate    =false;       //
        condition_variable  keyFrameUpdated;
        mutex               keyFrameUpdateMutex;

        // data to generate point clouds
        vector<KeyFrame*>       keyframes;
        vector<cv::Mat>         colorImgs,depthImgs;
        cv::Mat   depthImg,colorImg,mpose;

        vector<PointCloud::Ptr>   mvPointClouds;    //存储点云序列
        vector<PointCloud::Ptr>   mvPointCloudsForMatch;    //储存用于匹配点云序列
        vector<cv::Mat>   mvPosePointClouds;
        unsigned long int  mpointcloudID=0;

        mutex                   keyframeMutex;
        uint16_t                lastKeyframeSize =0;

        float cx=0,cy=0,fx=0,fy=0;

        Eigen::Matrix4f cvMat2Eigen(const cv::Mat &cvT);
    };
}
#endif // POINTCLOUDMAPPING_H