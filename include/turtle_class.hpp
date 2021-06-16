#ifndef __TURTLE_CLASS
#define __TURTLE_CLASS

#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <cppad/ipopt/solve.hpp>

struct Turtle
{
	turtlesim::Pose m_pose;

	Turtle();
	Turtle(const turtlesim::Pose& _pose);

	Turtle operator+(turtlesim::Pose _pose);
	Turtle operator-(turtlesim::Pose _pose);

	Turtle operator+=(turtlesim::Pose _pose);
	Turtle operator-=(turtlesim::Pose _pose);

	void callback(const turtlesim::Pose::ConstPtr& _ptr);

	~Turtle();
};

#endif