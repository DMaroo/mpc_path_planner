#ifndef __IPOPT_PLANNER
#define __IPOPT_PLANNER

#include <turtlesim/Pose.h> 
#include <geometry_msgs/Twist.h>
#include <cppad/ipopt/solve.hpp>

geometry_msgs::Twist get_velocity(turtlesim::Pose _runner, turtlesim::Pose _chaser, turtlesim::Pose _goal);
void initialise_twist(geometry_msgs::Twist& _twist);
bool close_to(const turtlesim::Pose& _pose1, const turtlesim::Pose& _pose2);
double theta_difference(double _reference, double _goal);

#endif