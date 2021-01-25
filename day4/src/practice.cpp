#include <ros/ros.h>
#include <iostream>
#include <algorithm>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <sensor_msgs/PointCloud2.h>

// How to fill up the block
class Practice:public ceres::SizedCostFunction<4, 1, 1, 1, 1>
{
    public:
    //Practice() = delete; // talk the initial value how to give and the method of "delete".
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {
        double x1(parameters[0][0]);double x2(parameters[1][0]);double x3(parameters[2][0]);double x4(parameters[3][0]);

        residuals[0] = x1 + 10*x2;
        residuals[1] = std::sqrt(5) * (x3-x4);
        residuals[2] = (x2-3*x3)*(x2-3*x3);
        residuals[3] = std::sqrt(10) * (x1-x4)*(x1-x4);

        if(jacobians)
        {
            if(jacobians[0])
            {
                Eigen::Map<Eigen::Vector4d> jacobian_pose0(jacobians[0]);
                jacobian_pose0.setZero();
                jacobian_pose0(0,0) = 1;

                jacobian_pose0(1,0) = 0;
                jacobian_pose0(2,0) = 0;
                jacobian_pose0(3,0) = 2*std::sqrt(10) *x1 - 2*std::sqrt(10)*x4;
            }
            if(jacobians[1])
            {
                Eigen::Map<Eigen::Vector4d > jacobian_pose1(jacobians[1]);
                jacobian_pose1.setZero();
                jacobian_pose1(0,0) = 10;
                jacobian_pose1(1,0) = 0;
                jacobian_pose1(2,0) = 2*x2-4*x3;
                jacobian_pose1(3,0) = 0;
            }
            if(jacobians[2])
            {
                Eigen::Map<Eigen::Vector4d> jacobian_pose3(jacobians[2]);
                jacobian_pose3.setZero();
                jacobian_pose3(0,0) = 0;
                jacobian_pose3(1,0) = std::sqrt(5);
                jacobian_pose3(2,0) = -4*x2+8*x3;
                jacobian_pose3(3,0) = 0;
            }
            if(jacobians[3])
            {
                Eigen::Map<Eigen::Vector4d > jacobian_pose3(jacobians[3]);
                jacobian_pose3.setZero();
                jacobian_pose3(0,0) = 0;
                jacobian_pose3(1,0) = -1*std::sqrt(5);
                jacobian_pose3(2,0) = 0;
                jacobian_pose3(3,0) = -2*std::sqrt(10)+2*std::sqrt(10)*x4;
            }
        }
        return true;
    }
};


int main(int argc, char **argv)
{
  ros::init (argc, argv, "depth");
  ros::NodeHandle nh("~");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);


  //Bulid the ceres
  ceres::Problem problem;
  ceres::LossFunction *loss_function;
  loss_function = new ceres::CauchyLoss(1.0);

  //add variable and given the intital value.
  double x1[1][1];
  double x2[1][1];
  double x3[1][1];
  double x4[1][1];
  x1[0][0] = 100;
  x2[0][0] = 789;
  x3[0][0] = 1314;
  x4[0][0] = 799;

  problem.AddParameterBlock(x1[0],1);
  problem.AddParameterBlock(x2[0],1);
  problem.AddParameterBlock(x3[0],1);
  problem.AddParameterBlock(x4[0],1);

  Practice *cost_function = new Practice();
  problem.AddResidualBlock(cost_function, NULL, x1[0], x2[0], x3[0], x4[0]);

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  //options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);


  std::cout<<"x state "<<x1[0][0]<<" "<<x2[0][0]<<" "<<x3[0][0]<<" "<<x4[0][0] <<std::endl;

  ros::spin();
  return 0;
}
