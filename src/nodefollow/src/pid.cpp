#include "nodefollow/myGoal.h"
#include "nodefollow/node_pid.h"

#define PI 3.141592
#define TOLERANCE 0.1  // Distance from the target distance, at which the distance will be considered as achieved.
#define TOLERANCE_ANGLE 0.2  // Differenc from target angle, which will be tolerated.
#define MAX_SPEED 1.0    // Maximum speed of robot.
#define MAX_A_SPEED 2.0    // Maximum angular speed of robot.
#define SUBSCRIBER_TOPIC_self "/self_GPS2XY"
#define SUBSCRIBER_TOPIC_front "/path_" + std::to_string(1)
#define SUBSCRIBER_TOPIC_front2 "/front_GPS2XY"
#define OPTIMALDIS 1.0

// bool checkMessage(NodePID* my_nodePID)
// {
//     if(my_nodePID->if_front_msg && my_nodePID->if_self_msg) return true;
//     return false;
// }

bool pidControl(NodePID* &my_nodePID)
{
    // if(checkMessage(my_nodePID))
    // {
    //     ROS_WARN("No get all msgs");
    //     return false;
    // }

    //获取跟踪点
    myGoal* the_target;
    double gooddis = DBL_MAX;
    std::vector<myGoal*> ref = my_nodePID->path;
    for(int i = 0; i < ref.size(); i++)
    {
        double nowdis = my_nodePID->vehicle_status->getDistance(ref[i]);
        if(fabs(nowdis - OPTIMALDIS) < fabs(gooddis - OPTIMALDIS))
        {
            gooddis = nowdis;
            the_target = ref[i];
        }
    }
    my_nodePID->target = the_target;

    //检查是否到达目标点
    if(my_nodePID->closeEnough(my_nodePID->target) == true)
    {
        ROS_INFO("TARGET ACHIEVED!");
        my_nodePID->speedCommand = 0.0;
        my_nodePID->angleCommand = 0.0;
        exit(0);
    }

    //首次运行时为
    if(my_nodePID->flag)
    {
        my_nodePID->last_dis = my_nodePID->vehicle_status->getDistance(my_nodePID->target);
        my_nodePID->last_ang = my_nodePID->vehicle_status->getAngle(my_nodePID->target);
        my_nodePID->last_time = my_nodePID->time;
    }
    my_nodePID->flag == false;

    //Saving to last
    my_nodePID->last_dis = my_nodePID->vehicle_status->getDistance(my_nodePID->target);
    my_nodePID->last_ang = my_nodePID->vehicle_status->getAngle(my_nodePID->target);
    my_nodePID->last_time = my_nodePID->time;

    my_nodePID->speedCommand = my_nodePID->calculatePID(my_nodePID->target, my_nodePID->vehicle_status->getDistance(my_nodePID->target) , my_nodePID->last_dis, 0.0, my_nodePID->D_KP, my_nodePID->D_KD, my_nodePID->D_KI, &(my_nodePID->dis_sum));
    my_nodePID->angleCommand = my_nodePID->calculatePID(my_nodePID->target, my_nodePID->vehicle_status->getAngle(my_nodePID->target) , my_nodePID->last_ang, 0.0, my_nodePID->A_KP, my_nodePID->A_KD, my_nodePID->A_KI, &(my_nodePID->ang_sum));

    //Invoking method for publishing message
    double finalspeeda = (fabs(my_nodePID->angleCommand) < my_nodePID->maxASpeed ? my_nodePID->angleCommand : copysign(my_nodePID->maxASpeed, my_nodePID->angleCommand));
    double finalspeed = (fabs(my_nodePID->speedCommand) < my_nodePID->maxSpeed ? my_nodePID->speedCommand : copysign(my_nodePID->maxSpeed, my_nodePID->speedCommand));
    printf("---CONTROL---\nnum.3:\nspeedCommand : %f ,angleCommand : %f\n", finalspeed, finalspeeda);

    return true;
}

//pidControl的纯寻迹版本
bool pidControl2(NodePID* &my_nodePID)
{
    // if(checkMessage(my_nodePID))
    // {
    //     ROS_WARN("No get all msgs");
    //     return false;
    // }

    // //获取跟踪点
    // myGoal* the_target;
    // double gooddis = DBL_MAX;
    // std::vector<myGoal*> ref = my_nodePID->path;
    // for(int i = 0; i < ref.size(); i++)
    // {
    //     double nowdis = my_nodePID->vehicle_status->getDistance(ref[i]);
    //     if(fabs(nowdis - OPTIMALDIS) < fabs(gooddis - OPTIMALDIS))
    //     {
    //         gooddis = nowdis;
    //         the_target = ref[i];
    //     }
    // }
    // my_nodePID->target = the_target;

    //检查是否到达目标点
    if(my_nodePID->closeEnough(my_nodePID->target) == true)
    {
        ROS_INFO("TARGET ACHIEVED!");
        my_nodePID->speedCommand = 0.0;
        my_nodePID->angleCommand = 0.0;
        exit(0);
    }

    //首次运行时为
    if(my_nodePID->flag)
    {
        my_nodePID->last_dis = my_nodePID->vehicle_status->getDistance(my_nodePID->target);
        my_nodePID->last_ang = my_nodePID->vehicle_status->getAngle(my_nodePID->target);
        my_nodePID->last_time = my_nodePID->time;
    }
    my_nodePID->flag == false;

    //Saving to last
    my_nodePID->last_dis = my_nodePID->vehicle_status->getDistance(my_nodePID->target);
    my_nodePID->last_ang = my_nodePID->vehicle_status->getAngle(my_nodePID->target);
    my_nodePID->last_time = my_nodePID->time;

    my_nodePID->speedCommand = my_nodePID->calculatePID(my_nodePID->target, my_nodePID->vehicle_status->getDistance(my_nodePID->target) , my_nodePID->last_dis, 0.0, my_nodePID->D_KP, my_nodePID->D_KD, my_nodePID->D_KI, &(my_nodePID->dis_sum));
    my_nodePID->angleCommand = my_nodePID->calculatePID(my_nodePID->target, my_nodePID->vehicle_status->getAngle(my_nodePID->target) , my_nodePID->last_ang, 0.0, my_nodePID->A_KP, my_nodePID->A_KD, my_nodePID->A_KI, &(my_nodePID->ang_sum));

    //Invoking method for publishing message
    double finalspeeda = (fabs(my_nodePID->angleCommand) < my_nodePID->maxASpeed ? my_nodePID->angleCommand : copysign(my_nodePID->maxASpeed, my_nodePID->angleCommand));
    double finalspeed = (fabs(my_nodePID->speedCommand) < my_nodePID->maxSpeed ? my_nodePID->speedCommand : copysign(my_nodePID->maxSpeed, my_nodePID->speedCommand));
    printf("---CONTROL---\nnum.3:\nspeedCommand : %f ,angleCommand : %f\n", finalspeed, finalspeeda);

    return true;
}


int main(int argc, char** argv)
{
    //休眠三秒为人操作作准备
    sleep(3);
    //初始化节点
    ros::init(argc,argv,"nodefollow");
    ros::NodeHandle n;

    //自车id
    int vehicleID;
    ////纯寻迹(1)与带规划寻迹(2)的状态信标
    //int followType = 0;

    //launch参数
    //n.param<int>("followType",followType,2);
	n.param<int>("vehicleID",vehicleID,1);

    // //初始化状态量为0
    // double distance = 0.0;
    // double angle = 0.0;

    //创建publisher用于与底盘通讯
    ros::Publisher pubMessage = n.advertise<geometry_msgs::Twist>("/cmd_vel", 500);

    /*构建pid算法对象来完成：
    * 控制量的计算 ： calculatePID
    * 是否到达终点的判断 ： closeEnough
    * 控制量的发布 ： publishMessage
    * 数据的储存及小车状态的更新（回调函数） ： messageCallback
    */
    NodePID* my_nodePID = new NodePID(pubMessage, TOLERANCE, TOLERANCE_ANGLE, MAX_SPEED, MAX_A_SPEED);

    /* 
     * 高级ros标准库同步处理多话题代码（暂不用）

    // // 需要用message_filter容器对两个话题的数据发布进行初始化，这里不能指定回调函数
    // message_filters::Subscriber<sensor_msgs::PointCloud2> sub_front(n, SUBSCRIBER_TOPIC_front, 500, ros::TransportHints().tcpNoDelay());     
    // message_filters::Subscriber<custom_messages::VehicleStatus> sub_self(n,SUBSCRIBER_TOPIC_self,500,ros::TransportHints().tcpNoDelay());

    // // 将两个话题的数据进行同步
    // typedef message_filters::sync_policies::ApproximateTime<custom_messages::VehicleStatus, sensor_msgs::PointCloud2> syncPolicy;
    // message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10),  sub_self, sub_front);  
    // // 指定一个回调函数，就可以实现两个话题数据的同步获取
    // sync.registerCallback(boost::bind(&NodePID::messageCallback, my_nodePID, _1, _2));

     */


    ros::Rate rate(500);
    bool status = ros::ok();

    //if(followType == 2)
    //{
        ros::Subscriber front_sub = n.subscribe(SUBSCRIBER_TOPIC_front, 10, &NodePID::frontCallback, my_nodePID);
        ros::Subscriber self_sub = n.subscribe(SUBSCRIBER_TOPIC_self, 10, &NodePID::selfCallback, my_nodePID);
        while (status)
        {
            ros::spinOnce();

            if(pidControl(my_nodePID))
            {
                my_nodePID->publishMessage(my_nodePID->angleCommand, my_nodePID->speedCommand);
            }

            rate.sleep();
            status = ros::ok();
        }
    //}
    // else if(followType == 1)
    // {
    //     ros::Subscriber front_sub = n.subscribe(SUBSCRIBER_TOPIC_front2, 10, &NodePID::frontCallback2, my_nodePID);
    //     ros::Subscriber self_sub = n.subscribe(SUBSCRIBER_TOPIC_self, 10, &NodePID::selfCallback, my_nodePID);
    //     while (status)
    //     {
    //         ros::spinOnce();

    //         if(pidControl2(my_nodePID))
    //         {
    //             my_nodePID->publishMessage(my_nodePID->angleCommand, my_nodePID->speedCommand);
    //         }

    //         rate.sleep();
    //         status = ros::ok();
    //     }
    // }



    // ros::spin();

    return 0;
}
