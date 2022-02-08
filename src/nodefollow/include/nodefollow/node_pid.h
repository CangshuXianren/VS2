/************************************************
@jyf
函数名称 ： node_pid
函数功能 ： pid跟随
备注 ： 
*************************************************/

#ifndef NODEPID_H
#define NODEPID_H

#include "myGoal.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <queue>
#include "custom_messages/VehicleStatus.h"
#include <sensor_msgs/PointCloud2.h>
#include "avoidance/common.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//多话题同步处理
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/thread/thread.hpp>

typedef pcl::PointXYZI PointType;

class NodePID
{
public:
    /*Constructor:
     *pub     Publisher,发送指令给小车
     *tol     Distance tolerance(m)
     *tolA    Angle tolerance(度)
     *dis     小车要跟随的距离
     *ang     小车要跟随的方向角
     *mSpeed  小车最大速度
     *mASpeed 小车最大角速度
     */
    NodePID(ros::Publisher pub,double tol, double tolA, double mSpeed, double mASpeed);

    ~NodePID();

    /*本函数发布小车的控制量：
     *angleCommand 角速度
     *speedCommand 线速度
     */
    void publishMessage(double angleCommand, double speedCommand);

    /*
     * 本函数从规划模块读取数据并将其处理为变量保存当前位姿
     * frontCallback    前车位置收集
     * frontCallback2   直接收集gps转换后的数据，用于纯寻迹
     * selfCallback     自车位置收集
     */
    void frontCallback(const sensor_msgs::PointCloud2::ConstPtr& front_msg);
    void frontCallback2(const custom_messages::VehicleStatus::ConstPtr& front_msg2);
    void selfCallback(const custom_messages::VehicleStatus::ConstPtr& self_msg);

    /*本函数计算是否足够接近目标
     *如果dis小于tol，则完成目标并且返回true
     *actual 当前状态量
     */
    bool closeEnough(myGoal* actual);

    /*本函数通过PID控制器计算动作干预
    actualVal   当前输出量
    lastVal     上一步的输出量
    ref         参考量
    Kp、Ki、Kd
    sum         sum of errors
    */
    double calculatePID(myGoal *actual, double actualVal, double lastVal, double ref, double Kp, double Ki, double Kd, double *sum);

//变量
    myGoal* vehicle_status; // 车辆当前状态
    myGoal* target;     //目标点
    double last_dis;    //上一时刻距离差
    double last_ang;    //上一时刻角度差
    ros::Time last_time;   //上一时刻时间
    ros::Time time;
    std::vector<myGoal*> path;   //跟踪路径

    // bool if_self_msg = true;
    // bool if_front_msg = true;

    double angleCommand;
    double speedCommand;

    //@调试
    double tolerance;     //dis容忍度
    double toleranceA;    //ang容忍度
    double maxSpeed;      //最大线速度
    double maxASpeed;     //最大角速度
    ros::Publisher pubMessage;  //发布角速度和线速度控制指令
    bool flag;      //判断当前是否为刚开始运行
    double dis_sum; //PID的sum of dis errors
    double ang_sum; //PID的sum of ang errors
    double gap; // 车间距

    //@调试
    const double D_KP = 0.3;  // dis控制器的P
    const double D_KI = 0.2;  // dis控制器的I
    const double D_KD = 0.0;  // dis控制器的D
    const double A_KP = 0.015; // ang控制器的P
    const double A_KI = 0.0; // ang控制器的I
    const double A_KD = 0.01; // ang控制器的D
};

#endif