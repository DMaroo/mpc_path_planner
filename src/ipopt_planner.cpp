#include "ipopt_planner.hpp"
#include <ros/ros.h>
#include <sstream>
#include <cmath>

#define DEL_T 0.5 // delta time between two consecutive path points
#define A_THRESH 0.4 // max acceleration (threshold)
#define T_THRESH 1.0 // max delta theta (threshold)
#define V_THRESH 2.0 // max velocity (threshold)
#define W_THRESH 2.0 // max omega (threshold)

double XG = 0, YG = 0; // variables to store goal position
extern double curr_acc; // current value of acceleration (declared in main.cpp)
turtlesim::Pose g_runner, g_chaser; // pose of the runner and chaser turtle

std::tuple<CppAD::AD<double>, CppAD::AD<double>> get_coordinates(CppAD::AD<double> x, CppAD::AD<double> y, CppAD::AD<double> v, CppAD::AD<double> a, CppAD::AD<double> t) // get the next set of coordinates using the current set of dynamical values
{
	CppAD::AD<double> r_x {x}, r_y {y};

	r_x = x + v*DEL_T*CppAD::cos(t) + 0.5*a*DEL_T*DEL_T*CppAD::cos(t); // kinematic equation in x direction
	r_y = y + v*DEL_T*CppAD::sin(t) + 0.5*a*DEL_T*DEL_T*CppAD::sin(t); // kinematic equation in y direction
	
	return std::make_tuple(r_x, r_y); // return the tuple
}

namespace
{
	using CppAD::AD;

	class FG_eval
	{
	public:
		typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

		void operator()(ADvector& fg, const ADvector& at)
		{
			assert(fg.size() == 12);
			assert(at.size() == 20);

			ADvector a(10), t(10);

			for (int i = 0; i < 20; i++)
			{
				if (i % 2 == 0)
				{
					a[i/2] = at[i]; // even indices correspond to acceleration values
				}
				else
				{
					t[(i - 1)/2] = at[i]; // odd indices correspond to theta values
				}
			}

			ADvector x(11), y(11), v(11); // vector for storing future values of x, y and v

			std::tuple<CppAD::AD<double>, CppAD::AD<double>> result; // variable to get result of get_coordinates function

			result = get_coordinates(g_runner.x, g_runner.y, g_runner.linear_velocity, curr_acc, g_runner.theta);

			x[0] = std::get<0>(result); // assign value to x[0]
			y[0] = std::get<1>(result); // assign value to y[0]
			v[0] = g_runner.linear_velocity + curr_acc*DEL_T; // update velocity

			for (int i = 1; i < 11; i++) // iterate for the remaining set of variables
			{
				result = get_coordinates(x[i - 1], y[i - 1], v[i - 1], a[i - 1], t[i - 1]);
				x[i] = std::get<0>(result); // assign value to x[i]
				y[i] = std::get<1>(result); // assign value to x[i]
				v[i] = v[i - 1] + a[i - 1]*DEL_T; // update velocity
			}

			for (int i = 10; i >= 0; i--)
			{
				fg[0] += pow(x[i] - XG, 2) + pow(y[i] - YG, 2); // objective function is the sum of squares of all the distances between the points and the goal
			}

			for (int i = 0; i < 11; i++)
			{
				fg[i + 1] = pow(x[i] - g_chaser.x, 2) + pow(y[i] - g_chaser.y, 2); // each of the constraints: distance of each point on the path from the chaser turtle
			}

			return;
		}
	};
}

double to_theta(double _theta) // convert a value to allowed values of theta: [-M_PI, M_PI]
{
	if (_theta < -M_PI)
	{
		_theta += 2*M_PI; 
	}
	else if (_theta > M_PI)
	{
		_theta -= 2*M_PI;
	}

	return _theta;
}

double theta_difference(double _goal, double _reference) // find the difference between two angles and return a valid difference
{
	_goal -= _reference;

	return to_theta(_goal);
}

geometry_msgs::Twist get_velocity(turtlesim::Pose _runner, turtlesim::Pose _chaser, turtlesim::Pose _goal) // return the computed twist velocity
{
	XG = _goal.x; // set the goal coordinates
	YG = _goal.y;

	g_runner = _runner; // set the pose values
	g_chaser = _chaser;

	using Dvector = CppAD::vector<double>;

	size_t nx = 20;
	size_t ng = 11;

	Dvector xi(nx);

	for (int i = 0; i < 10; i++)
	{
		xi[2*i] = curr_acc; // acceleration initial value (all assigned to the current acceleration)
		xi[2*i + 1] = _runner.theta; // all initial theta values assigned to the current runner's theta value
	}

	Dvector xl(nx), xu(nx);

	for (int i = 0; i < 20; i++)
	{
		if (i % 2 == 0)
		{
			xl[i] = -A_THRESH; // constrain the acceleration values
			xu[i] = A_THRESH;
		}
		else
		{
			xl[i] = _runner.theta - T_THRESH; // constrain the theta values
			xu[i] = _runner.theta + T_THRESH;
		}
	}

	Dvector gl(ng), gu(ng);

	for (int i = 0; i < 11; i++)
	{
		gl[i] = 3; // set the lower limit for distance between the chaser turtle and path points
		gu[i] = 1e10; // set the upper limit for the same (practically infinity)
	}

	FG_eval fg_eval;

	std::string options;

	options += "Integer print_level  0\n";
	options += "String sb  yes\n";
	options += "Integer max_iter  50\n"; // max iterations 50
	options += "Numeric max_cpu_time  0.1\n"; // max cpu time is 0.1 seconds
	options += "Sparse true  forward\n";
	options += "Sparse true  reverse\n";
	options += "Numeric tol  1e-3\n"; // tolerance is 10^-3
	options += "String derivative_test  second-order\n";
	options += "Numeric point_perturbation_radius  0.1\n";

	CppAD::ipopt::solve_result<Dvector> solution;

	CppAD::ipopt::solve<Dvector>(options, xi, xl, xu, gl, gu, fg_eval, solution);

	geometry_msgs::Twist solved;

	solved.angular.x = 0;
	solved.angular.y = 0;
	solved.angular.z = 0;
	solved.linear.x = 0;
	solved.linear.y = 0;
	solved.linear.z = 0;

	solved.angular.z = std::min(theta_difference(to_theta(solution.x[1]), _runner.theta)/DEL_T, W_THRESH); // find the angular velocity
	double a = solution.x[0];
	curr_acc = a; // update current acceleration

	solved.linear.x = std::min(_runner.linear_velocity + a*DEL_T, V_THRESH); // find the linear velocity

	return solved; // return the solution
}

void initialise_twist(geometry_msgs::Twist& _twist) // initialise all components of twist to 0
{
	_twist.angular.x = 0;
	_twist.angular.y = 0;
	_twist.angular.z = 0;

	_twist.linear.x = 0;
	_twist.linear.y = 0;
	_twist.linear.z = 0;
}

bool close_to(const turtlesim::Pose& _pose1, const turtlesim::Pose& _pose2) // check if two floating point numbers are "close enough"
{
	return (abs(_pose1.x - _pose2.x) <= 0.4 && abs(_pose1.y - _pose2.y) <= 0.4);
}
