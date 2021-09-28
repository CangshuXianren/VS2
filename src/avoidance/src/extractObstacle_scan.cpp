
#include <cmath>
#include <vector>

#include <avoidance/math_utils.h>
#include <avoidance/common.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/passthrough.h>

#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>

#include <stdio.h>
#include <dlfcn.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include <math.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_broadcaster.h>

#define PI 3.14159265f

using std::sin;
using std::cos;

ros::Subscriber subLaserCloud;

ros::Publisher  pubFilterCloud;
ros::Publisher  pubGroundCloud;
ros::Publisher  pubObjectCloud;
ros::Publisher  pubConeInfo;
ros::Publisher  pubConeVisual;

pcl::PointCloud<PointType>::Ptr laserCloudIn (new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr laserCloudIn2 (new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr laserCloudIn3 (new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr filterCloudLatest (new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr groundCloudLatest (new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr objectCloudLatest (new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr coneCouldLatest (new pcl::PointCloud<PointType>);

// fliter
pcl::PassThrough<PointType> pass1;
pcl::PassThrough<PointType> pass2;
pcl::PassThrough<PointType> pass3;

// Plane Model Segmentation
pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
// Create the segmentation object
pcl::SACSegmentation<PointType> seg;

pcl::ExtractIndices<PointType> extract;

pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
std::vector<pcl::PointIndices> cluster_indices;
pcl::EuclideanClusterExtraction<PointType> ec;

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{
    laserCloudIn->clear();
    laserCloudIn2->clear();
    laserCloudIn3->clear();
    filterCloudLatest->clear();
    groundCloudLatest->clear();
    objectCloudLatest->clear();
    coneCouldLatest->clear();
    
    // pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);
    sensor_msgs::PointCloud2 currentCloudMsg = *laserCloudMsg;
    pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);
    if (laserCloudIn->points.size())
    {
        ROS_ERROR("laserCloudIn has no points");
        return;
    }

    // fliter
    pass1.setInputCloud (laserCloudIn);
    pass1.setFilterFieldName ("x");
    pass1.setFilterLimits (0, 13.0);
    pass1.filter(*laserCloudIn2);

    pass2.setInputCloud (laserCloudIn2);
    pass2.setFilterFieldName ("y");
    pass2.setFilterLimits (-4.0, 4.0);
    pass2.filter (*laserCloudIn3);

    pass3.setInputCloud (laserCloudIn3);
    pass3.setFilterFieldName ("z");
    pass3.setFilterLimits (-1.5, 0.5);
    pass3.filter (*filterCloudLatest);

    // Plane Model Segmentation
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    // 距离阈值 单位m
    seg.setDistanceThreshold (0.05);
    seg.setInputCloud (filterCloudLatest);
    seg.segment (*inliers, *coefficients);
	
	// float A, B, C, D;
	// A = coefficients->values[0];
	// B = coefficients->values[1];
	// C = coefficients->values[2];
	// D = coefficients->values[3];
// 	ROS_INFO("Plane value : a = %f b = %f c = %f d = %f", 
// 			 coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);

    // 提取地面
    extract.setInputCloud (filterCloudLatest);
    extract.setIndices (inliers);
    extract.filter (*groundCloudLatest);
    // 提取除地面外的物体
    extract.setNegative (true);
    extract.filter (*objectCloudLatest);
    
    // cluster
    cluster_indices.clear();
    tree->setInputCloud(objectCloudLatest);
    //search strategy tree
    ec.setSearchMethod (tree);
    ec.setInputCloud (objectCloudLatest);
    ec.extract (cluster_indices);
    
    //push cone points
    float range = 0.0;
    PointType p;
    p.z = 1000.0;
    p.intensity = 0.0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
		{			
            range = sqrt(pow(objectCloudLatest->points[*pit].x, 2) + pow(objectCloudLatest->points[*pit].y, 2));
            if (range < p.z)
            {
                p.x = objectCloudLatest->points[*pit].x;
                p.y = objectCloudLatest->points[*pit].y;
                p.z = range;
            }
		}
        if (p.z == 1000.0) continue;

		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            range = sqrt(pow(objectCloudLatest->points[*pit].x - p.x, 2) + pow(objectCloudLatest->points[*pit].y - p.y, 2));
            if (range > p.intensity) p.intensity = range;
        }
        if (p.intensity == 0.0) continue;
        coneCouldLatest->points.push_back(p);
    }
    if (cluster_indices.size() != coneCouldLatest->points.size())
        ROS_WARN("cluster_indices.size != coneCouldLatest->points.size");
    if (filterCloudLatest->points.size()) ROS_WARN("filterCloudLatest has no points");
    if (groundCloudLatest->points.size()) ROS_WARN("groundCloudLatest has no points");
    if (objectCloudLatest->points.size()) ROS_WARN("objectCloudLatest has no points");
    
    //publishResult
    publishCloudMsg(pubConeInfo, *coneCouldLatest, "velodyne");
    
    publishCloudMsg(pubFilterCloud, *filterCloudLatest, "velodyne");
    publishCloudMsg(pubGroundCloud, *groundCloudLatest, "velodyne");
    publishCloudMsg(pubObjectCloud, *objectCloudLatest, "velodyne");

    publishPointsVisualMsg(pubConeVisual, *coneCouldLatest, 1.0, 0.753, 0.796, "velodyne");
//publishPointsVisualMsg(pubConeVisual, *cone_info_latest, 1.0, 0.0, 0.0, "/velodyne");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "extractObstacle");
    ros::NodeHandle nh;

    ec.setClusterTolerance(0.4);
    ec.setMinClusterSize(3);
    ec.setMaxClusterSize(300);
    
    subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>
				("/velodyne_points", 2, &laserCloudHandler);

    pubFilterCloud  = nh.advertise<sensor_msgs::PointCloud2> ("/filter", 1);
    pubGroundCloud  = nh.advertise<sensor_msgs::PointCloud2> ("/ground", 2);
    pubObjectCloud  = nh.advertise<sensor_msgs::PointCloud2> ("/object", 2);

    pubConeInfo   = nh.advertise<sensor_msgs::PointCloud2> ("/cone_info", 2);
    pubConeVisual = nh.advertise<visualization_msgs::Marker>("/cone_visual", 2);

    ROS_INFO("extractObstacle init successfully ...");
    
    ros::spin();

    return 0;
}
