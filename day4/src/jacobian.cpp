#include <ros/ros.h>
#include <queue>
#include <mutex>
#include <iostream>
#include <algorithm>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointXYZI PointType;
Eigen::Quaterniond temp_Q;
Eigen::Vector3d temp_t;
double param_pose[1][7];
std::mutex m_buf;
std::queue<sensor_msgs::PointCloud2ConstPtr> sourceBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> targetBuf;

void sourceHandler(const sensor_msgs::PointCloud2ConstPtr &sourceCloud)
{
  m_buf.lock();
  sourceBuf.push(sourceCloud);
  m_buf.unlock();
}

void targetHandler(const sensor_msgs::PointCloud2ConstPtr &targetCloud)
{
    m_buf.lock();
    sourceBuf.push(targetCloud);
    m_buf.unlock();
}

class Utility
{
  public:
    template <typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> deltaQ(const Eigen::MatrixBase<Derived> &theta)
    {
        typedef typename Derived::Scalar Scalar_t;

        Eigen::Quaternion<Scalar_t> dq;
        Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
        half_theta /= static_cast<Scalar_t>(2.0);
        dq.w() = static_cast<Scalar_t>(1.0);
        dq.x() = half_theta.x();
        dq.y() = half_theta.y();
        dq.z() = half_theta.z();
        return dq;
    }
};

class PoseLocalParameterization : public ceres::LocalParameterization
{
    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const
    {
        Eigen::Map<const Eigen::Vector3d> _p(x);
        Eigen::Map<const Eigen::Quaterniond> _q(x + 3);

        Eigen::Map<const Eigen::Vector3d> dp(delta);

        Eigen::Quaterniond dq = Utility::deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3));

        Eigen::Map<Eigen::Vector3d> p(x_plus_delta);
        Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 3);

        p = _p + dp;
        q = (_q * dq).normalized();

        return true;
    }
    virtual bool ComputeJacobian(const double *x, double *jacobian) const
    {
        Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
        j.topRows<6>().setIdentity();
        j.bottomRows<1>().setZero();

        return true;
    }
    virtual int GlobalSize() const { return 7; }
    virtual int LocalSize() const { return 6; }
};

class Transformation:public ceres::SizedCostFunction<3, 7>
{
public:
    Transformation() = delete;
    Transformation(Eigen::Vector3d source_, Eigen::Vector3d target_):source(source_),target(target_){}

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {
        Eigen::Quaterniond Q(parameters[0][0],parameters[0][1],parameters[0][2],parameters[0][3]);
        Eigen::Vector3d t(parameters[1][0],parameters[1][1],parameters[1][2]);

        Eigen::Vector3d resdiualt;
        resdiualt = target - Q * source + t;

        residuals[0] = resdiualt.x();
        residuals[1] = resdiualt.y();
        residuals[2] = resdiualt.z();

        if(jacobians)
        {
            if(jacobians[0])
            {
              // (3,4)指的是residual的size
              Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor> > jacobian_pose(jacobians[0]);
              jacobian_pose.setZero();

            }
        }
        return true;
    }
    Eigen::Vector3d source, target;
};

void vector2dobule()
{
    //quaternion
    param_pose[0][3] = temp_Q.w();
    param_pose[0][0] = temp_Q.x();
    param_pose[0][1] = temp_Q.y();
    param_pose[0][2] = temp_Q.z();

    //vector
    param_pose[0][4] = temp_t(0);
    param_pose[0][5] = temp_t(1);
    param_pose[0][6] = temp_t(2);
}

void double2vector()
{
    //quaternion
    temp_Q.w() =param_pose[0][3];
    temp_Q.x() =param_pose[0][0];
    temp_Q.y() =param_pose[0][1];
    temp_Q.z() =param_pose[0][2];

    //vector
    temp_t << param_pose[0][4], param_pose[0][5], param_pose[0][6];

}

void process()
{
    vector2dobule();

    pcl::PointCloud<PointType>::Ptr source(new pcl::PointCloud<PointType>()), target(new pcl::PointCloud<PointType>());
    pcl::fromROSMsg(*sourceBuf.back(),*source);
    pcl::fromROSMsg(*targetBuf.back(),*target);


    //Bulid the ceres
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    loss_function = new ceres::CauchyLoss(1.0);

    //add state parameter
    PoseLocalParameterization *local_parameterization = new PoseLocalParameterization;
    problem.AddParameterBlock(param_pose[0], 7, local_parameterization);

    //add resdiual
    //把特徵點塞進去就完成了
    for(size_t i =0; i<source->points.size();i++)
    {
        Eigen::Vector3d pointsource(source->points[i].x, source->points[i].y, source->points[i].z);
        Eigen::Vector3d pointarget(target->points[i].x, target->points[i].y, target->points[i].z);

        Transformation *cost_function = new Transformation(pointsource, pointarget);
        problem.AddResidualBlock(cost_function, NULL, param_pose[0]);
    }


    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    //options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    vector2dobule();
    std::cout<<"temp_Q : "<<temp_Q.coeffs()<<" temp_t : "<<temp_t.transpose()<<std::endl;
}

int main(int argc, char **argv)
{
    ros::init (argc, argv, "depth");
    ros::NodeHandle nh("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    ros::Subscriber source_sub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100, sourceHandler);
    ros::Subscriber target_sub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100, targetHandler);

    ros::spin();
    return 0;
}
