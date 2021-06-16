#include <iostream>
#include <termcolor/termcolor.hpp>
#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include "turtle_class.hpp"
#include "ipopt_planner.hpp"

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "planner");

	ros::NodeHandle planner_node;

	Turtle runner, chaser;

	std::cout << termcolor::bold << termcolor::yellow << "[*]" << termcolor::reset << " Setting up the publishers and subscribers..." << std::endl;

	ros::Subscriber runner_sub = planner_node.subscribe("/runner/pose", 100, &Turtle::callback, &runner);
	ros::Subscriber chaser_sub = planner_node.subscribe("/chaser/pose", 100, &Turtle::callback, &chaser);

	ros::Publisher runner_pub = planner_node.advertise<geometry_msgs::Twist>("/runner/cmd_vel", 100);

	std::cout << termcolor::bold << termcolor::green << "[+]" << termcolor::reset << " Planner nodes ready!" << std::endl;

	ros::Rate loop_rate = 100;

	geometry_msgs::Twist pub_vel;
	turtlesim::Pose goal;
	std::string state;

	goal.x = 10;
	goal.y = 10;

	std::cout << termcolor::bold << termcolor::yellow << "[*]" << termcolor::reset << " Entering main loop..." << std::endl;

	int flag = 1;

	while (true)
	{
		std::cout << std::endl << termcolor::color<100, 100, 100> << termcolor::bold << "[?]" << termcolor::reset << " To exit the program, enter \"" << termcolor::red << "exit" << termcolor::reset << "\" as any one of the inputs" << std::endl;

		std::cout << std::endl << termcolor::bold << termcolor::blue << "[=]" << termcolor::reset << "Please enter the goal position for the turtle (currently at (" << runner.m_pose.x << ", " << runner.m_pose.y << ")) (format: <x y>) : ";

		std::cin >> state;

		if (state == "exit")
		{
			break;
		}
		else
		{
			goal.x = atof(state.c_str());
		}

		std::cin >> state;

		if (state == "exit")
		{
			break;
		}
		else
		{
			goal.y = atof(state.c_str());
		}

		std::cout << std::endl << termcolor::bold << termcolor::yellow << "[*]" << termcolor::reset << " Turtle moving to (" << goal.x << ", " << goal.y << ")..." << std::endl;

		flag = 0;

		while (ros::ok())
		{
			if (flag == 0)
			{
				double curr_theta = atan2(goal.y - runner.m_pose.y, goal.x - runner.m_pose.x);

				if (theta_difference(curr_theta, runner.m_pose.theta) > 0.7)
				{
					initialise_twist(pub_vel);
					pub_vel.angular.z = -1;
				}
				else if (theta_difference(curr_theta, runner.m_pose.theta) < -0.7)
				{
					initialise_twist(pub_vel);
					pub_vel.angular.z = 1;
				}
				else
				{
					flag = 1;
				}
			}
			else if (close_to(runner.m_pose, goal))
			{
				initialise_twist(pub_vel);
				std::cout << termcolor::bold << termcolor::green << "[+]" << termcolor::reset << " Turtle reached the destination!" << std::endl;

				break;
			}
			else
			{
				pub_vel = get_velocity(runner.m_pose, chaser.m_pose, goal);
				flag = 1;
			}

			runner_pub.publish(pub_vel);

			ros::spinOnce();
			loop_rate.sleep();
		}
	}

	std::cout << std::endl << termcolor::bold << termcolor::red << "[-]" << termcolor::reset << " Exited the program!" << std::endl;
}