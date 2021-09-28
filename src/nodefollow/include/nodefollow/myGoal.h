/************************************************
@jyf
函数名称 ： myGoal
函数功能 ： 实现获取自身与前车的距离和角度并构造出一个点
备注 ： 
*************************************************/

#ifndef MYGOAL_H
#define MYGOAL_H

#include <math.h>
#include "ros/ros.h"

class myGoal
{
public:
    /*Constructor:
    xPos    x位置
    yPos    y位置
    alpha   robot朝向
    t   Time of measurement
    */
    myGoal(double xPos, double yPos, double yaw, double time);

    ~myGoal();

    //返回自身与前车构成的向量的方向角
    double getAngle(myGoal* target);

    //返回自身与前车距离
    double getDistance(myGoal* target);

//变量
    double x;
    double y;
//    ros::Time time; //Time. when the state was measured
	double time;
    double yaw;
};

#endif
