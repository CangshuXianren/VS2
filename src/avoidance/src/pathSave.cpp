
#include <fstream>
#include <ros/ros.h>
#include "custom_messages/VehicleStatus.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;

string filename;
int downsamplerate;

pcl::PointCloud<pcl::PointXYZI>::Ptr pathCloud(new pcl::PointCloud<pcl::PointXYZI>());

class pcdsss
{
public:
	pcdsss(){}
	~pcdsss()
	{
		if (pathCloud->points.size() > 0)
		{
		    string file_name = string("pcd/") + filename + string("_ori.pcd");
			string all_points_dir(string(ROOT_DIR) + file_name);
		    pcl::PCDWriter pcd_writer;
			cout << "pathCloud->size(): " << pathCloud->size() << endl;
		    pcd_writer.writeBinary(all_points_dir, *pathCloud);
			cout << "save done" << endl;
		}
		else
			cout << "no enough points" << endl;
	}
};

double selfPoseInitX, selfPoseInitY;
void selfPoseHandler(const custom_messages::VehicleStatus::ConstPtr& msg)
{
	static int num = 0, count = 0;
	static bool poseInit = false;

	if (count % downsamplerate != 0) return;

	pcl::PointXYZI selfPose;
	if (!poseInit)
	{
		selfPoseInitX = msg->xPos;
		selfPoseInitY = msg->yPos;
		poseInit = true;
		std::cout << "poseInit: " << selfPoseInitX << ", " << selfPoseInitY << std::endl;
	}
	else
	{
		selfPose.x = msg->xPos - selfPoseInitX;
		selfPose.y = msg->yPos - selfPoseInitY;
		selfPose.z = 0.0;
	//	selfPose.z = msg->theta;
		selfPose.intensity = num++;
		pathCloud->push_back(selfPose);
	}
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pathSave");
    ros::NodeHandle nh;

	nh.param<string>("filename",filename,"");
	nh.param<int>("downsamplerate",downsamplerate,10);

	pcdsss ps;

    ros::Subscriber subSelfPose = nh.subscribe<custom_messages::VehicleStatus>("/self_GPS2XY", 2, &selfPoseHandler);

	ros::spin();
}
