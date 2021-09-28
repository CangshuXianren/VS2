#include "self_gps2xy.h"
#define GPSMSGTYPE novatel_oem7_msgs::INSPVAX
#define SPANTOPIC "/novatel/oem7/inspvax"

GPS2XY::GPS2XY(ros::Publisher pub)
{
    this->pub = pub;
    return;
}

/* 经纬度转换为地心空间直角坐标系（应该不适用于无人车）

// void GPS2XY::transform(double l, double B, double xc, double yc, double yaw)
// {
//     l = l * M_PI /180;
//     B = B * M_PI /180;

//     double B0 =30* M_PI /180;

//     double N =0, e =0, a =0, b =0, e2 =0, K =0;
//     a =6378137;
//     b =6356752.3142;
//     e = sqrt(1- (b / a) * (b / a));
//     e2 = sqrt((a / b) * (a / b) -1);
//     double CosB0 = cos(B0);
//     N = (a * a / b) / sqrt(1+ e2 * e2 * CosB0 * CosB0);
//     K = N * CosB0;

//     double SinB = sin(B);

//     double tantan = tan(M_PI /4+ B /2);
//     double E2 = pow((1- e * SinB) / (1+ e * SinB), e /2);
//     double xx = tantan * E2;

//     xc = K * log(xx);
//     yc = K * l;

//     custom_messages::VehicleStatus msg;
//     msg.xPos = xc;
//     msg.yPos = yc;
//     msg.yaw = yaw;

//     pub.publish(msg);
//     return;
// }

 */

//经纬度坐标与高斯坐标的转换代码

// 高斯to经纬度
// // double y;      输入参数: 高斯坐标的横坐标，以米为单位
// // double x;      输入参数: 高斯坐标的纵坐标，以米为单位
// // short  DH;     输入参数: 带号，表示上述高斯坐标是哪个带的
// // double *L;     输出参数: 指向经度坐标的指针，其中经度坐标以秒为单位
// // double *B;     输出参数: 指向纬度坐标的指针，其中纬度坐标以秒为单位
// void GaussToGeo(double y, double x, short DH, double* L, double* B, double LP)
// {
//     double l0;    //  经差
//     double tf;    //  tf = tg(Bf0),注意要将Bf转换成以弧度为单位
//     double nf;    //  n = y * sqrt( 1 + etf ** 2) / c, 其中etf = e'**2 * cos(Bf0) ** 2
//     double t_l0;   //  l0，经差，以度为单位
//     double t_B0;   //  B0，纬度，以度为单位
//     double Bf0;    //  Bf0
//     double etf;    //  etf,其中etf = e'**2 * cos(Bf0) ** 2
//     double X_3;

//     double PI = 3.14159265358979;
//     double b_e2 = 0.0067385254147;
//     double b_c = 6399698.90178271;

//     X_3 = x / 1000000.00 - 3;      // 以兆米（）为单位
//     // 对于克拉索夫斯基椭球，计算Bf0
//     Bf0 = 27.11115372595 + 9.02468257083 * X_3 - 0.00579740442 * pow(X_3, 2)
//                    - 0.00043532572 * pow(X_3, 3) + 0.00004857285 * pow(X_3, 4)
//                    + 0.00000215727 * pow(X_3, 5) - 0.00000019399 * pow(X_3, 6);
//     tf = tan(Bf0 * PI / 180);       //  tf = tg(Bf),注意这里将Bf转换成以弧度为单位
//     etf = b_e2 * pow(cos(Bf0 * PI / 180), 2);   //  etf = e'**2 * cos(Bf) ** 2
//     nf = y * sqrt(1 + etf) / b_c;     //  n = y * sqrt( 1 + etf ** 2) / c
//     // 计算纬度，注意这里计算出来的结果是以度为单位的
//     t_B0 = Bf0 - (1.0 + etf) * tf / PI * (90.0 * pow(nf, 2)
//            - 7.5 * (5.0 + 3 * pow(tf, 2) + etf - 9 * etf * pow(tf, 2)) * pow(nf, 4)
//            + 0.25 * (61 + 90 * pow(tf, 2) + 45 * pow(tf, 4)) * pow(nf, 6));
//     // 计算经差，注意这里计算出来的结果是以度为单位的
//     t_l0 = (180 * nf - 30 * (1 + 2 * pow(tf, 2) + etf) * pow(nf, 3)
//              + 1.5 * (5 + 28 * pow(tf, 2) + 24 * pow(tf, 4)) * pow(nf, 5))
//              / (PI * cos(Bf0 * PI / 180));
//     l0 = (t_l0 * 3600.0);       //  将经差转成秒

//     if (LP == -1000)
//     {
//         *L = (double)((DH * 6 - 3) * 3600.0 + l0);  // 根据带号计算出以秒为单位的绝对经度，返回指针
//     }
//     else
//     {
//         *L = LP * 3600.0 + l0;  // 根据带号计算出以秒为单位的绝对经度，返回指针
//     }

//     *B = (double)(t_B0 * 3600.0);     //  将纬差转成秒，并返回指针
// }

//经纬度to高斯
// double jd;   输入参数: 地理坐标的经度，以秒为单位
// double wd;   输入参数: 地理坐标的纬度，以秒为单位
// short  DH;   输入参数: 三度带或六度带的带号（北京的三度带带号为39）

// 6度带带号=（经度+6°）/6取整 6度带中央经线=（6度带带号*6）-3
// 3度带带号=（经度+1.5°）/3取整 3度带中央经线=3度带带号*3
// 6度带中央经线经度的计算：当地中央经线经度=6°×当地带号-3°
// 3度带中央经线经度的计算：当地中央经线经度=3°×当地带号（中国陆地范围内带号小于23的肯定是6度带，大于等于24的肯定是3度带。）

void transform(double jd, double wd, short DH, short DH_width, double& y, double& x, double LP)
{
    double t;     //  t=tgB
    double L;     //  中央经线的经度
    double l0;    //  经差
    double jd_hd, wd_hd;  //  将jd、wd转换成以弧度为单位
    double et2;    //  et2 = (e' ** 2) * (cosB ** 2)
    double N;     //  N = C / sqrt(1 + et2)
    double X;     //  克拉索夫斯基椭球中子午弧长
    double m;     //  m = cosB * PI/180 * l0
    double tsin, tcos;   //  sinB,cosB
    double PI = 3.14159265358979;
    double b_e2 = 0.0067385254147;
    double b_c = 6399698.90178271;
    jd_hd = jd / 3600.0 * PI / 180.0;    // 将以秒为单位的经度转换成弧度
    wd_hd = wd / 3600.0 * PI / 180.0;    // 将以秒为单位的纬度转换成弧度

    // 如果不设中央经线（缺省参数: -1000），则计算中央经线，
    // 否则，使用传入的中央经线，不再使用带号和带宽参数
    //L = (DH - 0.5) * DH_width ;      // 计算中央经线的经度
    if (LP == -1000)
    {
        L = (DH - 0.5) * DH_width;      // 计算中央经线的经度
    }
    else
    {
        L = LP;
    }

    l0 = jd / 3600.0 - L;       // 计算经差
    tsin = sin(wd_hd);        // 计算sinB
    tcos = cos(wd_hd);        // 计算cosB
    // 计算克拉索夫斯基椭球中子午弧长X
    X = 111134.8611 / 3600.0 * wd - (32005.7799 * tsin + 133.9238 * pow(tsin, 3)
          + 0.6976 * pow(tsin, 5) + 0.0039 * pow(tsin, 7)) * tcos;
    et2 = b_e2 * pow(tcos, 2);      //  et2 = (e' ** 2) * (cosB ** 2)
    N = b_c / sqrt(1 + et2);      //  N = C / sqrt(1 + et2)
    t = tan(wd_hd);         //  t=tgB
    m = PI / 180 * l0 * tcos;       //  m = cosB * PI/180 * l0
    x = X + N * t * (0.5 * pow(m, 2)
              + (5.0 - pow(t, 2) + 9.0 * et2 + 4 * pow(et2, 2)) * pow(m, 4) / 24.0
              + (61.0 - 58.0 * pow(t, 2) + pow(t, 4)) * pow(m, 6) / 720.0);
    y = N * (m + (1.0 - pow(t, 2) + et2) * pow(m, 3) / 6.0
                    + (5.0 - 18.0 * pow(t, 2) + pow(t, 4) + 14.0 * et2
                       - 58.0 * et2 * pow(t, 2)) * pow(m, 5) / 120.0);

}


void GPS2XY::transformCallback(const GPSMSGTYPE::ConstPtr& gps_msg)
{
  double l = gps_msg->longitude;
  double b = gps_msg->latitude;
  double yaw = gps_msg->azimuth;
  double xc;
  double yc;
  transform(l*3600, b*3600, 39, 3, yc, xc, -1000);
  custom_messages::VehicleStatus msg;
  msg.xPos = xc;
  msg.yPos = yc;
  msg.yaw = yaw;

  pub.publish(msg);
  return;
}

int main(int argc, char **argv)  
{  
 
  ros::init(argc, argv, "self_GPS2XY");  

  ros::NodeHandle n;

  ros::Publisher pubMessage = n.advertise<custom_messages::VehicleStatus>("/self_GPS2XY", 10); 

  GPS2XY* self_GPS2XY = new GPS2XY(pubMessage);

  ros::Subscriber sub = n.subscribe(SPANTOPIC, 10, &GPS2XY::transformCallback, self_GPS2XY);

  ros::spin();  

  return 0;  
}  