#include "ipopt_planner.hpp"
#include <cmath>

#define DEL_T 1
#define DEL_X 1
#define THRESH_V 4
#define THRESH_W 4

double XR = 0, YR = 0, XC = 0, YC = 0, XG = 0, YG = 0, TR = 0;

namespace
{
	using CppAD::AD;

	class FG_eval
	{
	public:
		typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

		void operator()(ADvector& fg, const ADvector& xy)
		{
			assert(fg.size() == 21);
			assert(xy.size() == 20);

			ADvector x(10), y(10);

			for (int i = 0; i < 20; i++)
			{
				if (i % 2 == 0)
				{
					x[i/2] = xy[i];
				}
				else
				{
					y[(i - 1)/2] = xy[i];
				}
			}

			for (int i = 9; i >= 0; i--)
			{
				fg[0] += pow(x[i] - XG, 2) + pow(y[i] - YG, 2);
			}

			for (int i = 0; i < 10; i++)
			{
				fg[i + 1] = pow(x[i] - XC, 2) + pow(y[i] - YC, 2);
			}

			for (int i = 0; i < 9; i++)
			{
				fg[11 + i] = pow(x[i + 1] - x[i], 2) + pow(y[i + 1] - y[i], 2);
			}

			fg[20] = pow(x[0] - XR, 2) + pow(y[0] - YR, 2);

			return;
		}
	};
}

template <typename T>
int signum(T _x)
{
	assert(std::is_arithmetic<T>());

	if (_x > 0)
	{
		return 1;
	}
	else if (_x < 0)
	{
		return -1;
	}

	return 0;
}

double theta_difference(double _reference, double _goal)
{
	_goal -= _reference;

	if (_goal < -M_PI)
	{
		_goal += 2*M_PI; 
	}
	else if (_goal > M_PI)
	{
		_goal -= 2*M_PI;
	}

	return _goal;
}

geometry_msgs::Twist convert_to_twist(turtlesim::Pose _current, turtlesim::Pose _goal)
{
	double direct_theta = atan2(_goal.y - _current.y, _goal.x - _current.x);
	double theta_diff = theta_difference(_current.theta, direct_theta);

	geometry_msgs::Twist final_vel;

	final_vel.angular.z = (2*theta_diff)/DEL_T;

	if (abs(final_vel.angular.z) > THRESH_W)
	{
		final_vel.angular.z = signum(final_vel.angular.x)*THRESH_W;
	}

	double dist = (_current.x - _goal.x)*(_current.x - _goal.x) + (_current.y - _goal.y)*(_current.y - _goal.y);
	double radius = dist/(2*sin(theta_diff));

	double curve_len = 2*theta_diff*radius;

	final_vel.linear.x = curve_len/DEL_T;

	if (abs(final_vel.linear.x) > THRESH_V)
	{
		final_vel.linear.x = signum(final_vel.linear.x)*THRESH_V;
	}

	final_vel.angular.x = 0;
	final_vel.angular.y = 0;
	final_vel.linear.y = 0;
	final_vel.linear.z = 0;

	return final_vel;
}

geometry_msgs::Twist get_velocity(turtlesim::Pose _runner, turtlesim::Pose _chaser, turtlesim::Pose _goal)
{
	XR = _runner.x;
	YR = _runner.y;

	TR = _runner.theta;

	XC = _chaser.x;
	YC = _chaser.y;

	XG = _goal.x;
	YG = _goal.y;

	using Dvector = CppAD::vector<double>;

	size_t nx = 20;
	size_t ng = 20;

	Dvector xi(nx);

	xi[0] = XR;
	xi[1] = YR;

	double slope = atan2(XG - XR, YG - YR);

	for (int i = 1; i < 10; i++)
	{
		xi[2*i] = xi[2*i - 2] + 1*DEL_X*cos(slope);
		xi[2*i + 1] = xi[2*i - 1] + 1*DEL_X*sin(slope);
	}

	Dvector xl(nx), xu(nx);

	for (int i = 0; i < 20; i++)
	{
		xl[i] = -1e10;
		xu[i] = 1e10;
	}

	Dvector gl(ng), gu(ng);

	for (int i = 0; i < 10; i++)
	{
		gl[i] = 3;
		gu[i] = 1e10;
	}

	for (int i = 10; i < 20; i++)
	{
		gl[i] = 0;
		gu[i] = 2.0*DEL_X*DEL_X;
	}

	FG_eval fg_eval;

	std::string options;

	options += "Integer print_level  0\n";
	options += "String sb  yes\n";
	options += "Integer max_iter  50\n";
	options += "Numeric max_cpu_time  0.5\n";
	options += "Sparse true  forward\n";
	options += "Sparse true  reverse\n";
	options += "Numeric tol  1e-3\n";
	options += "String derivative_test  second-order\n";
	options += "Numeric point_perturbation_radius  0.1\n";

	CppAD::ipopt::solve_result<Dvector> solution;

	CppAD::ipopt::solve<Dvector>(options, xi, xl, xu, gl, gu, fg_eval, solution);

	turtlesim::Pose goal_pose;

	goal_pose.x = solution.x[0];
	goal_pose.y = solution.x[1];

	return convert_to_twist(_runner, goal_pose);
}

void initialise_twist(geometry_msgs::Twist& _twist)
{
	_twist.angular.x = 0;
	_twist.angular.y = 0;
	_twist.angular.z = 0;

	_twist.linear.x = 0;
	_twist.linear.y = 0;
	_twist.linear.z = 0;
}

bool close_to(const turtlesim::Pose& _pose1, const turtlesim::Pose& _pose2)
{
	return (abs(_pose1.x - _pose2.x) <= 0.2 && abs(_pose1.y - _pose2.y) <= 0.2);
}
