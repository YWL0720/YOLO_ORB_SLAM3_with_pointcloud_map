//
// Created by yuwenlu on 2022/7/2.
//
#include "PointCloudMapper.h"

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

PointCloudMapper::PointCloudMapper()
{
    mpGlobalMap = std::shared_ptr<PointCloud>(new PointCloud);
}

void PointCloudMapper::InsertKeyFrame(KeyFrame *kf, cv::Mat &imRGB, cv::Mat &imDepth)
{
    std::lock_guard<std::mutex> lck_loadKF(mmLoadKFMutex);
    mqKeyFrame.push(kf);
    mqRGB.push(imRGB.clone());
    mqDepth.push(imDepth.clone());
}

PointCloud::Ptr PointCloudMapper::GeneratePointCloud(KeyFrame *kf, cv::Mat &imRGB, cv::Mat &imDepth)
{
    PointCloud::Ptr pointCloud_temp(new PointCloud);

    for (int v=0; v<imRGB.rows; v++)
    {
        for (int u=0; u<imRGB.cols; u++)
        {
            cv::Point2i pt(u, v);
            bool IsDynamic = false;
            for (auto area : kf->mvDynamicArea)
                if (area.contains(pt)) IsDynamic = true;
            if (!IsDynamic)
            {
                float d = imDepth.ptr<float>(v)[u];
                if (d<0.01 || d>10) continue;
                PointT p;
                p.z = d;
                p.x = ( u - kf->cx) * p.z / kf->fx;
                p.y = ( v - kf->cy) * p.z / kf->fy;

                p.b = imRGB.ptr<cv::Vec3b>(v)[u][0];
                p.g = imRGB.ptr<cv::Vec3b>(v)[u][1];
                p.r = imRGB.ptr<cv::Vec3b>(v)[u][2];
                pointCloud_temp->push_back(p);
            }
        }
    }

    Eigen::Isometry3d T = ORB_SLAM3::Converter::toSE3Quat(kf->GetPose());
    PointCloud::Ptr pointCloud(new PointCloud);
    pcl::transformPointCloud(*pointCloud_temp, *pointCloud, T.inverse().matrix());
    pointCloud->is_dense = false;
    return pointCloud;
}

void PointCloudMapper::run()
{
    pcl::visualization::CloudViewer Viewer("Viewer");
    cout << endl << "PointCloudMapping thread start!" << endl;
    int ID = 0;
    while (1)
    {
        {
            std::lock_guard<std::mutex> lck_loadKFSize(mmLoadKFMutex);
            mKeyFrameSize= mqKeyFrame.size();
        }
        if (mKeyFrameSize != 0)
        {
            PointCloud::Ptr pointCloud_new(new PointCloud);
            pointCloud_new = GeneratePointCloud(mqKeyFrame.front(), mqRGB.front(), mqDepth.front());
            mqKeyFrame.pop();
            mqRGB.pop();
            mqDepth.pop();
            cout << "==============Insert No. " << ID << "KeyFrame ================" << endl;
            ID++;
            *mpGlobalMap += *pointCloud_new;
        }
        Viewer.showCloud(mpGlobalMap);
    }
}