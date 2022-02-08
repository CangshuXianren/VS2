#include"nodefollow/myGoal.h"

myGoal::myGoal(double xPos, double yPos, double yaw, double time)
{
    this->x = xPos;
    this->y = yPos;
    this->time = time;
    this->yaw = yaw;
}

myGoal::~myGoal()
{

}

double myGoal::getAngle(myGoal* target)
{
    return atan2((target->y - this->y),(target->x - this->x));
}

double myGoal::getDistance(myGoal* target)
{
    return sqrt(pow(target->y - this->y, 2.0) + pow(target->x - this->x, 2.0));
}