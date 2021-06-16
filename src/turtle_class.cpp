#include "turtle_class.hpp"

Turtle::Turtle()
{}

Turtle::Turtle(const turtlesim::Pose& _pose):
m_pose {_pose}
{}

Turtle Turtle::operator+(turtlesim::Pose _pose)
{
	turtlesim::Pose res;

	res.x = _pose.x + m_pose.x;
	res.y = _pose.y + m_pose.y;
	res.theta = _pose.theta + m_pose.theta;
	res.angular_velocity = _pose.angular_velocity + m_pose.angular_velocity;
	res.linear_velocity = _pose.linear_velocity + m_pose.linear_velocity;

	return res;
}

Turtle Turtle::operator-(turtlesim::Pose _pose)
{
	turtlesim::Pose res;

	res.x = _pose.x - m_pose.x;
	res.y = _pose.y - m_pose.y;
	res.theta = _pose.theta - m_pose.theta;
	res.angular_velocity = _pose.angular_velocity - m_pose.angular_velocity;
	res.linear_velocity = _pose.linear_velocity - m_pose.linear_velocity;

	return res;
}

Turtle Turtle::operator+=(turtlesim::Pose _pose)
{
	m_pose.x += _pose.x;
	m_pose.y += _pose.y;
	m_pose.theta += _pose.theta;
	m_pose.angular_velocity += _pose.angular_velocity;
	m_pose.linear_velocity += _pose.linear_velocity;

	return *this;
}

Turtle Turtle::operator-=(turtlesim::Pose _pose)
{
	m_pose.x -= _pose.x;
	m_pose.y -= _pose.y;
	m_pose.theta -= _pose.theta;
	m_pose.angular_velocity -= _pose.angular_velocity;
	m_pose.linear_velocity -= _pose.linear_velocity;

	return *this;
}

void Turtle::callback(const turtlesim::Pose::ConstPtr& _ptr)
{
	m_pose.x = _ptr->x;
	m_pose.y = _ptr->y;
	m_pose.theta = _ptr->theta;
	m_pose.angular_velocity = _ptr->angular_velocity;
	m_pose.linear_velocity = _ptr->linear_velocity;
}

Turtle::~Turtle()
{}