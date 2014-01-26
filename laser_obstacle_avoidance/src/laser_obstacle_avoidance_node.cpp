/*
 * laser_obstacle_avoidance_.cpp
 *
 *  Created on: 26 Jan 2014
 *      Author: Murilo F. M.
 *       Email: muhrix@gmail.com
 *
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#include <boost/bind.hpp>

#include <algorithm>

static const double cruisespeed = 0.75;
static const double avoidspeed = 0.3;
static const double avoidturn = 0.8;
static const double minfrontdistance = 1.2; // 0.6
static const double stopdist = 0.6;
static const int avoidduration = 50;

int avoidcount = 0;

void laserScanCallback(ros::Publisher& pub, geometry_msgs::Twist& cmd_msg,
		double& v_max, double& w_max, const sensor_msgs::LaserScanConstPtr& scan_data) {

	const sensor_msgs::LaserScan::_ranges_type& scan = scan_data->ranges;
	uint32_t sample_count = scan.size();

	bool obstruction = false;
	bool stop = false;

	// find the closest distance to the left and right and check if
	// there's anything in front
	double minleft = 1e6;
	double minright = 1e6;

	for (uint32_t i = 0; i < sample_count; i++) {
		if ((i > (sample_count / 3))
				&& (i < (sample_count - (sample_count / 3)))
				&& scan[i] < minfrontdistance) {
			obstruction = true;
		}

		if (scan[i] < stopdist) {
			stop = true;
		}

		if (i > sample_count / 2)
			minleft = std::min(minleft, (double)scan[i]);
		else
			minright = std::min(minright, (double)scan[i]);
	}

	if (obstruction || stop || (avoidcount > 0)) {
		cmd_msg.linear.x = (stop ? 0.0 : avoidspeed);

		/* once we start avoiding, select a turn direction and stick
		 with it for a few iterations */
		if (avoidcount < 1) {
			avoidcount = random() % avoidduration + avoidduration;

			if (minleft < minright) {
				cmd_msg.angular.z = -avoidturn;
			} else {
				cmd_msg.angular.z = avoidturn;
			}
		}
		avoidcount--;
	} else {
		avoidcount = 0;
		cmd_msg.linear.x = cruisespeed;
		cmd_msg.angular.z = 0.0;
	}
	// publish cmd_msgs on topic cmd_vel using publisher pub
	pub.publish(cmd_msg);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "laser_obstacle_avoidance");
	ROS_INFO("Starting laser obstacle avoidance behaviour");

	ros::NodeHandle my_node;
	geometry_msgs::Twist cmd;
	double vx, wz;

	my_node.param("vx", vx, 1.0);
	my_node.param("wz", wz, 0.8);

	ros::Publisher pub_vel = my_node.advertise<geometry_msgs::Twist>("cmd_vel", 1);

	ros::Subscriber sub_laser = my_node.subscribe<sensor_msgs::LaserScan>
	("scan", 1, boost::bind(&laserScanCallback, boost::ref(pub_vel), boost::ref(cmd),
			boost::ref(vx), boost::ref(wz), _1));

	ros::spin();

	ROS_INFO("Shutting down laser obstacle avoidance behaviour");

	return 0;
}

