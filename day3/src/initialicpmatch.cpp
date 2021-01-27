#include <ros/ros.h>
#include <iostream>
#include <thread>
#include <queue>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <math.h>
#include <mutex>

typedef pcl::PointXYZI PointType;
typedef std::pair<sensor_msgs::PointCloud2ConstPtr ,geometry_msgs::PoseStampedConstPtr> CombinedData;

ros::Publisher pub_icp;
sensor_msgs::PointCloud2 pointBefore, pointAfter;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointAllBuf;
std::queue<geometry_msgs::PoseStampedConstPtr> optiBuf;
std::queue<CombinedData> measurements;

std::vector<pcl::PointCloud<PointType>::Ptr> PointCloud;
std::vector<Eigen::Quaterniond> R_Opti;
std::vector<Eigen::Vector3d> t_Opti;
ros::Publisher pub_pointBefore, pub_pointAfter;

std::mutex m_buf, com_buf, process_buf;
bool initial = false;
int frame_count = 0;

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloud)
{
    m_buf.lock();
    pointAllBuf.push(laserCloud);
    m_buf.unlock();
    pub_pointBefore.publish(pointBefore);
    pub_pointAfter.publish(pointAfter);
}

void opti_callback(const geometry_msgs::PoseStampedConstPtr &opti)
{
    m_buf.lock();
    optiBuf.push(opti);
    m_buf.unlock();
}

CombinedData getMeasurement()
{
    CombinedData measurement;
    if(!pointAllBuf.empty() && !optiBuf.empty())
    {
        measurement = std::make_pair(pointAllBuf.back(), optiBuf.back());
    }
    return measurement;
}

void icpmatch(pcl::PointCloud<PointType>::Ptr &i_frame, pcl::PointCloud<PointType>::Ptr &j_frame,
              Eigen::Matrix4f Initial_guess)
{
    std::vector<int> indices1,indices2;
    pcl::removeNaNFromPointCloud(*i_frame, *i_frame, indices1);
    pcl::removeNaNFromPointCloud(*j_frame, *j_frame, indices2);
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    pcl::PointCloud<PointType>::Ptr Final(new pcl::PointCloud<PointType>());

    icp.setMaxCorrespondenceDistance(10);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-10);
    icp.setEuclideanFitnessEpsilon(1e-10);
    icp.setRANSACIterations(0);

    icp.setInputSource(j_frame);
    icp.setInputTarget(i_frame);
    icp.align(*Final,Initial_guess);

    if (icp.hasConverged())
    {
        ROS_WARN_STREAM("finish");
    }

}

void process()
{
    pcl::PointCloud<PointType>::Ptr pointcloud(new pcl::PointCloud<PointType>());
    // 將ros message格式轉成pcl
    pcl::fromROSMsg(*measurements.front().first, *pointcloud);

    PointCloud.push_back(pointcloud);

    Eigen::Quaterniond temp_Q;
    Eigen::Vector3d temp_t;

    temp_Q.w() = measurements.back().second->pose.orientation.w;
    temp_Q.x() = measurements.back().second->pose.orientation.x;
    temp_Q.y() = measurements.back().second->pose.orientation.y;
    temp_Q.z() = measurements.back().second->pose.orientation.z;

    temp_t(0) = measurements.back().second->pose.position.x;
    temp_t(1) = measurements.back().second->pose.position.y;
    temp_t(2) = measurements.back().second->pose.position.z;

    R_Opti.push_back(temp_Q);
    t_Opti.push_back(temp_t);

    if(frame_count != 0)
    {
        Eigen::Quaterniond Q;
        Eigen::Vector3d t;

        Q = R_Opti[frame_count-1].inverse() * R_Opti[frame_count];
        t = R_Opti[frame_count-1].inverse() * (t_Opti[frame_count]-t_Opti[frame_count-1]);

        Eigen::Matrix4f Init = Eigen::Matrix4f::Identity();
        Eigen::Matrix3d xxx = Q.toRotationMatrix();
        Init.block<3,3>(0,0) = xxx.transpose().cast<float>();
        Init.block<3,1>(0,3) = t.cast<float>();

        icpmatch(PointCloud[frame_count-1], PointCloud[frame_count], Init);

        pcl::toROSMsg(*PointCloud[frame_count-1], pointBefore);
        pcl::toROSMsg(*PointCloud[frame_count], pointAfter);

        pointBefore.header.frame_id = "/map";
        pointAfter.header.frame_id = "/map";
    }
    frame_count++;
    measurements.pop();
}

void command()
{
    while(ros::ok)
    {
      char c =  getchar();
      if( c == 's')
      {
        com_buf.lock();
        measurements.push(getMeasurement());
        com_buf.unlock();
        process_buf.lock();
        process();
        process_buf.unlock();
      }
    }
}

int main(int argc, char **argv)
{
  ros::init (argc, argv, "depth");
  ros::NodeHandle nh("~");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  ros::Subscriber pcl_sub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100, laserCloudHandler);
  ros::Subscriber opti_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/RigidBody1/pose", 100, opti_callback);

  pub_pointBefore = nh.advertise<sensor_msgs::PointCloud2>("/PointBefore",100);
  pub_pointAfter = nh.advertise<sensor_msgs::PointCloud2>("/PointTarget",100);

  pointBefore.header.frame_id = "/map";
  pointAfter.header.frame_id = "/map";

  std::thread command_process{command};
  ros::spin();
  return 0;
}
