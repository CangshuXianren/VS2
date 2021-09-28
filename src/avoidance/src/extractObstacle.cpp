#include <cmath>
#include <vector>

#include <avoidance/math_utils.h>
#include <avoidance/common.h>
#include "custom_messages/VehicleStatus.h"
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
#include <pcl/kdtree/kdtree_flann.h>
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

//宏定义
#define LIDARPCTOPIC "/velodyne_points"

using namespace std;

ros::Publisher  pubFilterCloud;
ros::Publisher  pubGroundCloud;
ros::Publisher  pubObjectCloud;
ros::Publisher  pubConeInfo;
ros::Publisher  pubConeVisual;
ros::Publisher  pubPathPointOri;
ros::Publisher  pubPathPointAvo;
ros::Publisher  pubObject;
ros::Publisher  pubPath;

pcl::PointCloud<PointType>::Ptr pathCloud(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudIn (new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr laserCloudIn2 (new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr laserCloudIn3 (new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr filterCloudLatest (new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr groundCloudLatest (new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr objectCloudLatest (new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr coneCouldLatest (new pcl::PointCloud<PointType>);

pcl::EuclideanClusterExtraction<PointType> ec;
std::vector<pcl::PointIndices> cluster_indices;

bool hasCloud = false, offline, pubcloud, followpath;

pcl::PointCloud<PointType>::Ptr pathCouldLatest (new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr pathCouldAvo (new pcl::PointCloud<PointType>);
pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
const double TURN_DIS = 5.0;
const double TURN_DIS_2 = 25.0;

pcl::PointXYZI selfPose;
Twist poseCurrent;

void processPathPoint();
void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{
    laserCloudIn->clear();
    laserCloudIn2->clear();
    laserCloudIn3->clear();
    filterCloudLatest->clear();
    groundCloudLatest->clear();
    objectCloudLatest->clear();
    coneCouldLatest->clear();
    
    pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);
    // sensor_msgs::PointCloud2 currentCloudMsg = *laserCloudMsg;
    // pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);
    if (laserCloudIn->points.size() == 0)
    {
        ROS_ERROR("laserCloudIn has no points");
        return;
    }

    // ROS_INFO("fliter ...");

    // fliter
    pcl::PassThrough<PointType> pass1;
    pcl::PassThrough<PointType> pass2;
    pcl::PassThrough<PointType> pass3;
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

    // ROS_INFO("Segmentation ...");

    // Plane Model Segmentation
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<PointType> seg;
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
    pcl::ExtractIndices<PointType> extract;
    extract.setInputCloud (filterCloudLatest);
    extract.setIndices (inliers);
    extract.filter (*groundCloudLatest);
    // 提取除地面外的物体
    extract.setNegative (true);
    extract.filter (*objectCloudLatest);
    
    // ROS_INFO("cluster ...");
    std::vector<pcl::PointIndices> cluster_indices;
    // cluster
    pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
    // 
    // pcl::EuclideanClusterExtraction<PointType> ec;
    cluster_indices.clear();
    tree->setInputCloud(objectCloudLatest);
    //search strategy tree
    ec.setSearchMethod (tree);
    ec.setInputCloud (objectCloudLatest);
    ec.extract (cluster_indices);
    
    //push cone points
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        PointType p;
        p.z = 0.0; // distance
        p.intensity = 0.0; // radio
		int num = 0;
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
		{
			p.x += objectCloudLatest->points[*pit].x;
			p.y += objectCloudLatest->points[*pit].y;
			num++;
		}
        p.x = p.x / float(num);
        p.y = p.y / float(num);

		float range = 0.0;
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            range = pow(objectCloudLatest->points[*pit].x - p.x, 2) + pow(objectCloudLatest->points[*pit].y - p.y, 2);
            if (range > p.intensity) p.intensity = range;
        }
        if (p.intensity == 0.0)
		{
			ROS_WARN("no radio");
			continue;
		}
		p.intensity = sqrt(p.intensity);
		p.z = sqrt(p.x*p.x + p.y*p.y);
        coneCouldLatest->points.push_back(p);
        std::cout << p.x << ' ' << p.y << ' ' << p.z << ' ' << p.intensity << std::endl;
    }
    if (cluster_indices.size() != coneCouldLatest->points.size())
        ROS_WARN("cluster_indices.size != coneCouldLatest->points.size");
    if (filterCloudLatest->points.size() == 0) ROS_WARN("filterCloudLatest has no points");
    if (groundCloudLatest->points.size() == 0) ROS_WARN("groundCloudLatest has no points");
    if (objectCloudLatest->points.size() == 0) ROS_WARN("objectCloudLatest has no points");
    // ROS_INFO("coneCouldLatest->points.size: %d", coneCouldLatest->points.size());
    
	hasCloud = true;
    // ROS_INFO("publishResult ...");

	processPathPoint();

    //publishResult
	if (pubcloud)
	{
		publishCloudMsg(pubConeInfo, *coneCouldLatest, "velodyne");
		
		publishCloudMsg(pubFilterCloud, *filterCloudLatest, "velodyne");
		publishCloudMsg(pubGroundCloud, *groundCloudLatest, "velodyne");
		publishCloudMsg(pubObjectCloud, *objectCloudLatest, "velodyne");

		publishPointsVisualMsg(pubConeVisual, *coneCouldLatest, 1.0, 0.753, 0.796, "velodyne");
	//publishPointsVisualMsg(pubConeVisual, *cone_info_latest, 1.0, 0.0, 0.0, "/velodyne");
	}
}

void generatePathOffline()
{
	PointType p;
	p.x = 0.0;
	p.y = 0.0;
	p.z = 0.0;
	p.intensity = 0.0;
	for (float i = -2.0; i < 12; i = i + 0.1)
	{
		p.y = i;
		p.x = i*i/20.0 + 1;
//		pathCouldLatest->push_back(p);
		pathCloud->push_back(p);
	}
	selfPose.x = 0.0;
	selfPose.y = 0.0;
	selfPose.z = 0.0;
	selfPose.intensity = 1.0;
//	std::cout << "path offline size: " << pathCloud->points.size() << std::endl;
}

double objx, objy, obji;
void generateObjOffline()
{
	coneCouldLatest->clear();
	PointType p;
	p.x = objx;
	p.y = objy;
	p.z = 5.5;
	p.intensity = obji;
	coneCouldLatest->push_back(p);
}

int pathIdx = 0;
double pathRadio, searchRadio, idxDiff;
bool getPath()
{
	if (selfPose.intensity != 1.0) return false;

    pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
    std::vector<int> k_indices;
    std::vector<float> k_sqr_distances;

    tree->setInputCloud(pathCloud);
    int numtree = tree->radiusSearch(selfPose, pathRadio, k_indices, k_sqr_distances);

	if (numtree == 0) { ROS_WARN("kdtree get no path points"); return false; }

	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	inliers->indices = k_indices;

	pcl::PointCloud<PointType>::Ptr pathCloudExt (new pcl::PointCloud<PointType>);
    pcl::ExtractIndices<PointType> extract;
    extract.setInputCloud (pathCloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*pathCloudExt);

	std::sort(pathCloudExt->points.begin(), pathCloudExt->points.end(), 
		[](PointType a, PointType b) {return a.intensity < b.intensity;});

	if (pathCloudExt->points.back().intensity - pathCloudExt->points.front().intensity > idxDiff)
	{
		if (abs(pathCloudExt->points.front().intensity - pathIdx) < 
			abs(pathCloudExt->points.back().intensity - pathIdx))
		{
			while (pathCloudExt->points.back().intensity - pathCloudExt->points.front().intensity > idxDiff)
				pathCloudExt->points.erase(pathCloudExt->points.end());
		}
		else
		{
			while (pathCloudExt->points.back().intensity - pathCloudExt->points.front().intensity > idxDiff)
				pathCloudExt->points.erase(pathCloudExt->points.begin());
		}
		if (pathCloudExt->points.size() == 0) 
			{ ROS_WARN("pathCloudExt remain null: %d", pathIdx); return false; }
	}
	*pathCouldLatest = *pathCloudExt;
	pathIdx = (pathCloudExt->points.back().intensity - pathCloudExt->points.front().intensity) * 0.5;
	return true;
}

double avoidcoff;
void processPathPoint()
{
//	if (!hasCloud) return;
	
	pathCouldLatest->clear();
	pathCouldAvo->clear();
	// push back
	
	if (offline)
	{
		generatePathOffline();
		generateObjOffline();
		pathCouldLatest = pathCloud;
	}
	else if (getPath() != true) return;

	if (!offline) for (auto &p : *pathCouldLatest) coneAntiTrans(p, p, poseCurrent);
	
	if (coneCouldLatest->points.size() > 0)
	{
		float minDis = 100.0;
		PointType pobj;
		for (auto &p : *coneCouldLatest)
		{
			if (p.intensity < minDis)
			{
				pobj = p;
				minDis = p.z;
			}
		}
		pobj.z = 0.0;
cout << "pobj: " << pobj.x << ", " << pobj.y << ", " << pobj.z << ", " << pobj.intensity << endl;
        std::vector<int>   k_indices;
        std::vector<float> k_sqr_distances;
		tree->setInputCloud(pathCouldLatest);
		int numtree = tree->nearestKSearch(pobj, 1, k_indices, k_sqr_distances);
		if (numtree == 0)
		{
			ROS_WARN("no nearest path point");
			return;
		}
		else
		{
			PointType ppath;
			ppath = pathCouldLatest->points[k_indices[0]];
cout << "ppath: " << ppath.x << ", " << ppath.y << ", " << ppath.z << ", " << ppath.intensity << endl;
			Eigen::Vector3d vpath (Eigen::Vector3d(ppath.x, ppath.y, 0.0));
			Eigen::Vector3d vobj (Eigen::Vector3d(pobj.x, pobj.y, 0.0));
			Eigen::Vector3d vpo = vobj - vpath;
			vpo = vpo / sqrt(vpo[0]*vpo[0] + vpo[1]*vpo[1]);
			Eigen::Vector3d vfinal = vobj - avoidcoff*pobj.intensity*vpo;
cout << "vfinal: " << vfinal[0] << ", " << vfinal[1] << ", " << vfinal[2] << endl;
			// rviz
			
			Eigen::Vector3d vpf = vfinal - vpath;
			Eigen::Vector3d vnew;
			PointType pathpoint;
			pathpoint.z = 0.0;
			for (auto &p : *pathCouldLatest)
			{
				double dis2 = pow(p.x-ppath.x, 2) + pow(p.y-ppath.y, 2);
				if (dis2 < TURN_DIS_2)
//					vnew = Eigen::Vector3d(p.x, p.y, 0.0) + vpf * (TURN_DIS - sqrt(dis2)) / TURN_DIS;
					vnew = Eigen::Vector3d(p.x, p.y, 0.0) + vpf / (1 + exp(2*(-TURN_DIS/2+sqrt(dis2))));
				else
					vnew = Eigen::Vector3d(p.x, p.y, 0.0);
				// push back
				
				if (vnew[1] <= 0) continue;
				pathpoint.x = vnew[0];
				pathpoint.y = vnew[1];
				pathCouldAvo->push_back(pathpoint);
			}
		}
	}
	
	if (!offline) publishCloudMsg(pubPath, *pathCouldAvo, "velodyne");
	else
		while(1)
		{
			publishPointsVisualMsg(pubPathPointOri, *pathCouldLatest, 1.0, 0.0, 0.0, "/velodyne");
			publishPointsVisualMsg(pubPathPointAvo, *pathCouldAvo, 0.0, 1.0, 0.0, "/velodyne");
			publishObjVisualMsg(pubObject, *coneCouldLatest, 0.0, 0.0, 1.0, 1.0, 1.0, "/velodyne");
		}
}

double selfPoseInitX, selfPoseInitY, selfPoseInitT;
void selfPoseHandler(const custom_messages::VehicleStatus::ConstPtr& msg)
{
	selfPose.x = msg->xPos - selfPoseInitX;
	selfPose.y = msg->yPos - selfPoseInitY;
	selfPose.z = msg->yaw - selfPoseInitT;
	selfPose.intensity = 1.0;

	poseCurrent.pos.x() = selfPose.x;
	poseCurrent.pos.y() = selfPose.y;
	poseCurrent.pos.z() = 0.0;
	poseCurrent.rot_x = 0.0;
	poseCurrent.rot_y = 0.0;
	poseCurrent.rot_z = selfPose.z;

	if (followpath)
	{
		static int num = 0;
		if (num % 10 == 0) processPathPoint();
		num++;
	}
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "extractObstacle");
    ros::NodeHandle nh;

	string filename;
	int vehicleID;

	nh.param<bool>("offline",offline,true);
	nh.param<bool>("pubcloud",pubcloud,false);
	nh.param<bool>("followpath",followpath,false);
	nh.param<double>("objx",objx,0.5);
	nh.param<double>("objy",objy,5.0);
	nh.param<double>("obji",obji,1.0);
	nh.param<double>("avoidcoff",avoidcoff,1.44);
	nh.param<string>("filename",filename,"");
	nh.param<double>("pathRadio",pathRadio,10.0);
	nh.param<double>("searchRadio",searchRadio,3.0);
	nh.param<double>("selfPoseInitX",selfPoseInitX,0.0);
	nh.param<double>("selfPoseInitY",selfPoseInitY,0.0);
	nh.param<double>("selfPoseInitT",selfPoseInitT,0.0);
	nh.param<double>("idxDiff",idxDiff,1000.0);
	nh.param<int>("vehicleID",vehicleID,0);

	if (!offline)
	{
		string file_name = string("pcd/") + filename + string("_ori.pcd");

		if (pcl::io::loadPCDFile<pcl::PointXYZI> (file_name, *pathCloud) == -1)
		{
			PCL_ERROR ("Couldn't read file path.pcd \n");
			return (-1);
		}
		std::cout << "Loaded " << pathCloud->size () << " data points from path.pcd" << std::endl;
	}

    ec.setClusterTolerance(0.4);
    ec.setMinClusterSize(3);
    ec.setMaxClusterSize(300);

	selfPose.intensity = 0.0;
    
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>
				(LIDARPCTOPIC, 2, &laserCloudHandler);
    ros::Subscriber subSelfPose = nh.subscribe<custom_messages::VehicleStatus>
				("/self_GPS2XY", 2, &selfPoseHandler);

    pubFilterCloud  = nh.advertise<sensor_msgs::PointCloud2> ("/filter", 1);
    pubGroundCloud  = nh.advertise<sensor_msgs::PointCloud2> ("/ground", 2);
    pubObjectCloud  = nh.advertise<sensor_msgs::PointCloud2> ("/object", 2);

    pubConeInfo   = nh.advertise<sensor_msgs::PointCloud2> ("/cone_info", 2);
    pubConeVisual = nh.advertise<visualization_msgs::Marker>("/cone_visual", 2);
	
	pubPathPointOri = nh.advertise<visualization_msgs::Marker>("/path_ori", 2);
	pubPathPointAvo = nh.advertise<visualization_msgs::Marker>("/path_avo", 2);
	pubObject = nh.advertise<visualization_msgs::Marker>("/path_object", 2);

	if (!offline) pubPath = nh.advertise<sensor_msgs::PointCloud2>("/path_" + std::to_string(vehicleID), 2);

    ROS_INFO("extractObstacle init successfully ...");

	if (!offline) processPathPoint();
    
    ros::spin();

    return 0;
}
