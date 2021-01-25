#include "day1/ncrl.h"

//這邊就不用在加static了
void ncrl::set(ncrl::Data &p, Eigen::Vector3d linear_a, Eigen::Vector3d angular_)
{
 p.acc = linear_a;
 p.ang_vel = angular_;
}
