#include "nodefollow/node_pid.h"
#include <math.h>
#define PI 3.141592

NodePID::NodePID(ros::Publisher pub, double tol, double tolA, double mSpeed, double mASpeed)
{
    tolerance = tol;
    toleranceA = tolA;
    maxSpeed = mSpeed;
    maxASpeed = mASpeed;
    pubMessage = pub;
    flag = true;
    dis_sum = 0.0;
    ang_sum = 0.0;
    last_dis = 0.0;
    last_ang = 0.0;
    last_time = ros::Time::now();
}

NodePID::~NodePID()
{
}

double NodePID::calculatePID(myGoal *target, double actualVal, double lastVal, double ref, double Kp, double Kd, double Ki, double *sum)
{
    double speed = 0;
    double error = actualVal - ref;
    double previousError = lastVal - ref;
    double dt = target->time - last_time.toSec();
    double derivative = (error - previousError) / dt;
    *sum = *sum + error * dt;
    speed = Kp * error + Kd * derivative + Ki * (*sum);
    return speed;
}

bool NodePID::closeEnough(myGoal *target)
{
    double df_dis, df_ang;
    df_dis = this->vehicle_status->getDistance(target);
    df_ang = this->vehicle_status->getAngle(target);
    if (fabs(df_dis) > tolerance) return false;
    if (
        fabs(df_ang) > toleranceA &&
        fabs(df_ang) + 2*PI > toleranceA &&
        fabs(df_ang) - 2*PI > toleranceA
    )
        return false;
    return true;
}

//Publisher
void NodePID::publishMessage(double angleCommand, double speedCommand)
{
    //preparing message
    geometry_msgs::Twist msg;

    msg.linear.x = speedCommand;
    msg.angular.z = angleCommand;

    //publishing message
    pubMessage.publish(msg);
}


Twist poseCurrent;
void NodePID::frontCallback(const sensor_msgs::PointCloud2::ConstPtr& front_msg)
{
    //接受目标点信息
    // this->if_front_msg = true;
    // myGoal* target = new myGoal(front_msg->xPos, front_msg->yPos, 2.0*asin(front_msg->yaw), front_msg->header.stamp.toSec());
    this->time = front_msg->header.stamp;

    pcl::PointCloud<PointType>::Ptr pathCloud (new pcl::PointCloud<PointType>);
    pcl::fromROSMsg(*front_msg, *pathCloud);

    for (auto &p : *pathCloud)
		coneTrans(p, p, poseCurrent);

    this->path.clear();
    myGoal* p = new myGoal(0.0, 0.0, 0.0, 0.0);
	for (int i=1; i<pathCloud->points.size()-1; i++)
	{
		(*p).x = pathCloud->points[i].x;
		(*p).y = pathCloud->points[i].y;
		(*p).yaw = atan( (pathCloud->points[i+1].y - pathCloud->points[i-1].y) / 
			(pathCloud->points[i+1].x - pathCloud->points[i-1].x) );
        this->path.push_back(p);
	}
    delete p;
}

void NodePID::frontCallback2(const custom_messages::VehicleStatus::ConstPtr& front_msg2)
{
    //接受目标点信息
    // this->if_front_msg = true;
    // myGoal* target = new myGoal(front_msg->xPos, front_msg->yPos, 2.0*asin(front_msg->yaw), front_msg->header.stamp.toSec());
    this->time = front_msg2->header.stamp;
    this->target->x = front_msg2->xPos;
    this->target->y = front_msg2->yPos;
    this->target->yaw = front_msg2->yaw;
}

void NodePID::selfCallback(const custom_messages::VehicleStatus::ConstPtr& self_msg)
{
    // this->if_self_msg = true;

    this->vehicle_status->x = self_msg->xPos;
    this->vehicle_status->y = self_msg->yPos;
    this->vehicle_status->yaw = self_msg->yaw;

	poseCurrent.pos.x() = self_msg->xPos;
	poseCurrent.pos.y() = self_msg->yPos;
	poseCurrent.pos.z() = 0.0;
	poseCurrent.rot_x = 0.0;
	poseCurrent.rot_y = 0.0;
	poseCurrent.rot_z = self_msg->yaw;
}
