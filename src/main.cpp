#include <iostream>
#include <termcolor/termcolor.hpp>
#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include "turtle_class.hpp"
#include "ipopt_planner.hpp"

double curr_acc = 1;

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "planner"); // initialise a node named "planner"

	ros::NodeHandle planner_node; // get a node handle

	Turtle runner, chaser; // create two Turtle classes, one for chaser, other for the runner

	std::cout << termcolor::bold << termcolor::yellow << "[*]" << termcolor::reset << " Setting up the publishers and subscribers..." << std::endl; // user prompt

	ros::Subscriber runner_sub = planner_node.subscribe("/runner/pose", 100, &Turtle::callback, &runner); // set up a subscriber for runner's pose
	ros::Subscriber chaser_sub = planner_node.subscribe("/chaser/pose", 100, &Turtle::callback, &chaser); // set up a subscriber for chaser's pose

	ros::Publisher runner_pub = planner_node.advertise<geometry_msgs::Twist>("/runner/cmd_vel", 100); // set up a velocity publisher for runner

	std::cout << termcolor::bold << termcolor::green << "[+]" << termcolor::reset << " Planner nodes ready!" << std::endl;

	ros::Rate loop_rate = 100; // set loop rate of 100Hz

	geometry_msgs::Twist pub_vel; // variable to be published
	turtlesim::Pose goal; // goal pose
	std::string state;

	goal.x = 10; // initialize goal pose to (10, 10)
	goal.y = 10;

	std::cout << termcolor::bold << termcolor::yellow << "[*]" << termcolor::reset << " Entering main loop..." << std::endl;

	int flag; // create a flag variable

	while (true)
	{
		std::cout << std::endl << termcolor::color<100, 100, 100> << termcolor::bold << "[?]" << termcolor::reset << " To exit the program, enter \"" << termcolor::red << "exit" << termcolor::reset << "\" as any one of the inputs" << std::endl; // info regarding exiting

		std::cout << std::endl << termcolor::bold << termcolor::blue << "[=]" << termcolor::reset << " Please enter the goal position for the turtle (currently at (" << runner.m_pose.x << ", " << runner.m_pose.y << ")) (format: <x y>) : "; // ask the user for goal pose

		std::cin >> state;

		if (state == "exit") // check if input is exit
		{
			break;
		}
		else
		{
			goal.x = atof(state.c_str()); // assign to goal.x
		}

		std::cin >> state;

		if (state == "exit")
		{
			break;
		}
		else
		{
			goal.y = atof(state.c_str()); // assign to goal.y
		}

		std::cout << std::endl << termcolor::bold << termcolor::yellow << "[*]" << termcolor::reset << " Turtle moving to (" << goal.x << ", " << goal.y << ")..." << std::endl;

		flag = 0; // set the flag to 0

		while (ros::ok()) // enter the ros loop
		{
			if (flag == 0) // if flag is 0
			{
				double curr_theta = atan2(goal.y - runner.m_pose.y, goal.x - runner.m_pose.x); // find the current theta

				if (theta_difference(runner.m_pose.theta, curr_theta) > 0.7) // get the theta difference, if greater than a certain threshold
				{
					initialise_twist(pub_vel); // initialize the publishing velocity
					pub_vel.angular.z = -1; // set the omega to -1
				}
				else if (theta_difference(runner.m_pose.theta, curr_theta) < -0.7) // if lower than a certain threshold
				{
					initialise_twist(pub_vel); // initialize the publishing velocity
					pub_vel.angular.z = 1; // set the omega to +1
				}
				else // if within the threshold
				{
					flag = 1; // set the flag to 1
				}
			}
			else if (close_to(runner.m_pose, goal)) // if the runner is close to the goal
			{
				initialise_twist(pub_vel); // reset the pub_vel
				std::cout << termcolor::bold << termcolor::green << "[+]" << termcolor::reset << " Turtle reached the destination!" << std::endl; // output success message

				break; // break
			}
			else
			{
				pub_vel = get_velocity(runner.m_pose, chaser.m_pose, goal); // otherwise get the velocity
				flag = 1; // set the flag to 1 (doesn't make any difference ideally speaking, but just in case)
			}

			runner_pub.publish(pub_vel); // publish the velocity

			ros::spinOnce(); // process the callbacks
			loop_rate.sleep(); // sleep for the desired rate
		}
	}

	std::cout << std::endl << termcolor::bold << termcolor::red << "[-]" << termcolor::reset << " Exited the program!" << std::endl; // output exit message
}