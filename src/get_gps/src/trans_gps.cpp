#include "trans_gps.h"
#define GPSMSGTYPE novatel_oem7_msgs::INSPVAX
#define SPANTOPIC "/novatel/oem7/inspvax"

Trans_GPS::Trans_GPS(ros::Publisher pub)
{
    this->pub = pub;
    return;
}

void Trans_GPS::gpsfixCallback(const GPSMSGTYPE::ConstPtr& gps_msg){

    double latitude, longitude, yaw, trans_stamp;
 
    latitude = gps_msg -> latitude;
    longitude = gps_msg -> longitude;
    yaw = gps_msg -> azimuth;
    trans_stamp = gps_msg->header.stamp.toSec();

    char cp[1000];
    std_msgs::String msg;
    sprintf(cp,",%f,%f,%f,%f,", latitude, longitude, yaw, trans_stamp);
    msg.data=cp;
    pub.publish(msg); 
};


int main(int argc, char **argv)  
{  
 
  ros::init(argc, argv, "trans_GPS");  

  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<std_msgs::String>("/PreNlink_linktrack_data_transmission", 10); 

  Trans_GPS* myTrans_GPS = new Trans_GPS(pub);

  ros::Subscriber sub = n.subscribe(SPANTOPIC, 200, &Trans_GPS::gpsfixCallback, myTrans_GPS); 

  ros::spin();  

  return 0;  
}  


