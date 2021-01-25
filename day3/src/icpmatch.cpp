#include <iostream>
#include <thread>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <math.h>

typedef pcl::PointXYZI PointType;

ros::Publisher pub_pointBefore, pub_pointTarget, pub_icp, pub_original;
sensor_msgs::PointCloud2 pointBefore, pointTarget, point_icp, point_original;

double photo(double x)
{
  double y;
  y = std::sqrt(5-(x*x));

  return y;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr addpoint()
{
  //Creates two pcl::PointCloud<pcl::PointXYZI> boost shared pointers and initializes them
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZI>);

  // Fill in the CloudIn data
  cloud_in->width    = 50;
  cloud_in->height   = 10;
  cloud_in->is_dense = false;
  cloud_in->points.resize (cloud_in->width * cloud_in->height);
  for (size_t i = 0; i < cloud_in->points.size (); ++i)
  {
      cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
      cloud_in->points[i].y = photo(1024 * rand () / (RAND_MAX + 1.0f));
      cloud_in->points[i].z = 1;
  }
  return cloud_in;
}

void transformation( const PointType* const pi, PointType* const po,Eigen::Quaterniond temp_q,Eigen::Vector3d temp_t)
{

    Eigen::Vector3d point_B(pi->x, pi->y, pi->z);
    Eigen::Vector3d point_A = temp_q * point_B + temp_t;
    po->x = point_A.x();
    po->y = point_A.y();
    po->z = point_A.z();
    po->intensity = pi->intensity;
}

void getTransformation(Eigen::Matrix4f icpTransformation, Eigen::Quaterniond &R, Eigen::Vector3d &t)
{
  R = icpTransformation.block<3,3>(0,0).cast<double>();
  t = icpTransformation.block<3,1>(0,3).cast<double>();
}


void process()
{
  //Creates two pcl::PointCloud<pcl::PointXYZI> boost shared pointers and initializes them
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_original (new pcl::PointCloud<pcl::PointXYZI>);

  //temp是拿來rviz顯示用的
  pcl::PointCloud<pcl::PointXYZI>::Ptr temp (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr tempA (new pcl::PointCloud<pcl::PointXYZI>);

  //設置偏移量
  Eigen::Quaterniond Qdif(0.5,0.5,-0.5,0.5);
  Eigen::Vector3d tdif(1.5,0,0);

  cloud_in = addpoint();
  *temp = *cloud_in;
  *cloud_original = *cloud_in;


  for (size_t i = 0; i < cloud_in->points.size (); ++i)
  {
    PointType pointSel = cloud_in->points[i];
    transformation(&pointSel ,&pointSel, Qdif, tdif);
    cloud_out->points.push_back(pointSel);
  }

  //creates an instance of an IterativeClosestPoint and gives it some useful information
  pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
  icp.setInputCloud(cloud_in);
  icp.setInputTarget(cloud_out);

  // align就會自動將cloud_in對齊到cloud_out了
  // 幫我多加一個initial guess
  icp.align(*cloud_in);

  if(icp.hasConverged())
  {
    //Obtain the Euclidean fitness score (e.g., sum of squared distances from the source to the target)
    std::cout << "First score: " <<icp.getFitnessScore() << std::endl;
    std::cout << "----------------------------------------------------------"<< std::endl;

    //Get the final transformation matrix estimated by the registration method.
    std::cout << icp.getFinalTransformation() << std::endl;
  }

  Eigen::Quaterniond R;
  Eigen::Vector3d t;

  Eigen::Matrix4f Initial_guess;
  Initial_guess = icp.getFinalTransformation();
  getTransformation(Initial_guess, R, t);

  while(true)
  {
    Eigen::Vector3d tmep_t = t/10.0;

    for(double j =0;j <11; j++)
    {
      tempA->clear();
      for(size_t i =0; i<temp->points.size(); ++i)
      {
        PointType pointSel = temp->points[i];
        transformation(&pointSel ,&pointSel, R, tmep_t*j);
        tempA->points.push_back(pointSel);
      }

      pcl::toROSMsg(*tempA,pointBefore);
      pcl::toROSMsg(*cloud_out,pointTarget);
      pcl::toROSMsg(*cloud_in,point_icp);
      pcl::toROSMsg(*cloud_original, point_original);

      pointBefore.header.frame_id = "/map";
      pointTarget.header.frame_id = "/map";
      point_icp.header.frame_id = "/map";
      point_original.header.frame_id = "/map";

      pub_pointBefore.publish(pointBefore);
      pub_pointTarget.publish(pointTarget);
      pub_icp.publish(point_icp);
      pub_original.publish(point_original);

      ros::Duration(0.5).sleep();
    }
  }

}

int main(int argc, char **argv)
{
  ros::init (argc, argv, "depth");
  ros::NodeHandle nh("~");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  pub_pointBefore = nh.advertise<sensor_msgs::PointCloud2>("/PointBefore",100);
  pub_pointTarget = nh.advertise<sensor_msgs::PointCloud2>("/PointTarget",100);
  pub_icp = nh.advertise<sensor_msgs::PointCloud2>("/PointIcp",100);
  pub_original = nh.advertise<sensor_msgs::PointCloud2>("/PointOriginal",100);

  process();
}
