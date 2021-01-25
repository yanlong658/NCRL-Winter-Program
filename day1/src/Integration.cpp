#include <ros/ros.h>
#include <Eigen/Geometry>
#include <sensor_msgs/Imu.h>
#include <eigen3/Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include "day1/ncrl.h"

double last_tims_stampe = 0;
double tims_stampe;
double dt = 0;
bool flag = false;

geometry_msgs::Point p;

Eigen::Vector3d distance;
Eigen::Vector3d velocity;
double sigma;

ncrl::Data data;

Eigen::Matrix3d B,rotation;

//must use const
void imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    if (!flag)
        flag = true;
  if(last_tims_stampe != 0)
  {
    tims_stampe = msg->header.stamp.toSec();
    dt = tims_stampe - last_tims_stampe;
    last_tims_stampe = tims_stampe;
  }
  else
  {
    last_tims_stampe = msg->header.stamp.toSec();
    rotation << 1,0,0
               ,0,1,0
               ,0,0,1;
  }

  //std::cout<<"dt"<<std::setprecision(10)<<dt<<std::endl;
  //std::cout<<"tims_stampe"<<tims_stampe;
  Eigen::Vector3d linear_a(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  Eigen::Vector3d angular_(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  ncrl::set(data,linear_a,angular_);

  sigma = std::sqrt(std::pow(data.ang_vel(0),2)+std::pow(data.ang_vel(1),2)+std::pow(data.ang_vel(2),2));

  B << 0 , -1 * data.ang_vel(2) * dt , data.ang_vel(1) * dt
     , data.ang_vel(2) * dt , 0 , -1 * data.ang_vel(0) * dt
     , -1 * data.ang_vel(1) * dt , data.ang_vel(0) * dt , 0;


  rotation = rotation * (Eigen::Matrix3d::Identity() + ((std::sin(sigma)/sigma) * B) + ((1 - std::cos(sigma))/std::pow(sigma,2)) * (B*B));
  data.acc = rotation * data.acc;

  velocity(0)= velocity(0) +  data.acc.x() * dt ;
  distance(0) = distance(0) + velocity(0)* dt ;
  //std::cout<<"dt1\t "<<std::setprecision(10)<<dt<<std::endl;
  //std::cout<<"here"<<std::endl;

  velocity(1)= velocity(1) + data.acc.y() * dt ;
  distance(1) = distance(1) + velocity(1)* dt ;

  p.x = distance(0);
  p.y = distance(1);


}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "path");
    ros::NodeHandle nh;
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    //因為數據關係他最低位數只有到15521278210 - 15521278090 ＝ 12 second
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("imu/data", 10,imu_cb);

    //visualization_msgs
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "WORLD";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "points_and_lines";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = 1;
    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;
    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    //設置監聽速率
    ros::Rate loop_rate(200);

    while(ros::ok)
    {
        if (flag){
            line_strip.header.stamp = ros::Time::now();
            line_strip.points.push_back(p);
            marker_pub.publish(line_strip);
        }


        ros::spinOnce();
        loop_rate.sleep();
    }

    //只要執行到監聽,他就會把他監聽完
    //ros::spin()
    return 0;
}
