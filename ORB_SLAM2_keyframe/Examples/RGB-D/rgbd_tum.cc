#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include<System.h>

using namespace std;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);
void SaveData(string &savepath,
            vector<cv::Mat> kf_image,
            vector<cv::Mat> kf_depth,
            vector<cv::Mat> kf_pose,
            vector<double> kf_stamp);
int main(int argc, char **argv)
{
    vector<cv::Mat> kf_image;
    vector<cv::Mat> kf_depth;
    vector<cv::Mat> kf_pose;
    vector<double> kf_stamp;
    vector<cv::Mat> keyframe_image_list;

    if(argc != 5)
    {
        cerr << endl << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association" << endl;
        return 1;
    }

    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strAssociationFilename = string(argv[4]);
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }
    
    int Viewer,Mapping,Relocation;
    string setfile = string(argv[2]);
    cv::FileStorage fs(setfile.c_str(),cv::FileStorage::READ);
    Viewer = fs["Viewer"];
    Mapping = fs["Mapping"];
    Relocation = fs["Relocation"];
    string MapPath = fs["MapPath"];
    string Trajectory = fs["Trajectory"];
    if(Mapping)
        cout<<"Mapping mode"<<endl;
    else
        cout<<"location mode"<<endl;

    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,Viewer);

    if(Relocation)
    {
        string orb_map = MapPath + ".bin";
        SLAM.LoadMap(orb_map.c_str());
        SLAM.ActivateLocalizationMode();
    }

    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat imRGB, imD;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image and depthmap from file
        imRGB = cv::imread(string(argv[3])+"/"+vstrImageFilenamesRGB[ni],CV_LOAD_IMAGE_UNCHANGED);
        imD = cv::imread(string(argv[3])+"/"+vstrImageFilenamesD[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        bool isKeyFrame,mapping;
        mapping = false;
        cv::Mat Tcw = SLAM.TrackRGBD(imRGB,imD,tframe,isKeyFrame,mapping); 
        // if(isKeyFrame)
        // {
        //     kf_image.push_back(imRGB.clone());
        //     kf_depth.push_back(imD.clone());
        //     kf_pose.push_back(Tcw.clone());
        //     kf_stamp.push_back(tframe);
        // }    
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    SLAM.Shutdown();

    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    if(Mapping)
        Trajectory = Trajectory + "Mapping";
    else
        Trajectory = Trajectory + "Location";

    Trajectory = Trajectory + ".txt";
    SLAM.SaveKeyFrameTrajectoryTUM(Trajectory.c_str()); 


    if(Mapping)
    {
        string orb_map = MapPath + ".bin";
        SLAM.SaveMap(orb_map.c_str());
    }


    // cout<<"kf_image.size = "<<kf_image.size()<<endl;
    // cout<<"kf_depth.size = "<<kf_depth.size()<<endl;
    // cout<<"kf_pose.size = "<<kf_pose.size()<<endl;
    // SaveData(savepath,kf_image,kf_depth,kf_pose,kf_stamp);
    return 0;
}

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);
        }
    }
}

void SaveData(string &savepath,
            vector<cv::Mat> kf_image,
            vector<cv::Mat> kf_depth,
            vector<cv::Mat> kf_pose,
            vector<double> kf_stamp)
{
    string rgb_txt = savepath + "/rgb.txt";
    string depth_txt = savepath + "/depth.txt";
    string pose_txt = savepath + "/KeyFrameTrajectory.txt";
    string rbg_path = savepath + "/rgb/";
    string depth_path = savepath + "/depth/";
    ofstream rgbf,depthf,posef;
    rgbf.open(rgb_txt.c_str());
    depthf.open(depth_txt.c_str());
    posef.open(pose_txt.c_str());
    rgbf<<"timestamp rgb"<<endl;
    depthf<<"timestamp depth"<<endl;
    posef<<"timestamp x y z qx qy qz qw"<<endl;
    for (size_t i = 0; i < kf_image.size(); i++)
    {
        rgbf<<fixed;
        depthf<<fixed;
        posef<<fixed;
        stringstream ss;
        ss<<fixed<<kf_stamp[i];
        string rgb_name = rbg_path + ss.str() + ".png";
        string depth_name = depth_path + ss.str() + ".png";
        rgbf<<fixed<<"rgb/"<<ss.str()<<".png"<<endl;
        depthf<<fixed<<"depth/"<<ss.str()<<".png"<<endl;
        cv::imwrite(rgb_name,kf_image[i]);
        // cout<<fixed<<"rbg_name = "<<rgb_name.c_str()<<endl;
        cv::imwrite(depth_name,kf_depth[i]);

        Eigen::Isometry3d Tcw;
        for (size_t a = 0; a < 4; a++)
        {
            for (size_t b = 0; b < 4; b++)
            {
                Tcw(a,b) = kf_pose[i].at<float>(a,b);
            }
        }
        Eigen::Isometry3d Twc = Tcw.inverse();
        Eigen::Matrix3d Rwc = Twc.rotation();
        Eigen::Quaterniond q = Eigen::Quaterniond(Rwc);
        Eigen::Vector3d twc = Twc.translation();
        posef<<ss.str()<<" "<<twc.x()<<" "<<twc.y()<<" "<<twc.z()<<" "<<
        q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<endl;
    }
    rgbf.close();
    depthf.close();
    posef.close();
    cout<<"Map Data Save done!"<<endl;
}