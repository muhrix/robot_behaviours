/*
 * laser_obstacle_avoidance_.cpp
 *
 *  Created on: 26 Jan 2014
 *      Author: Murilo F. M.
 *       Email: muhrix@gmail.com
 *
 * Adapted from wander.cc found at https://github.com/rtv/Stage
 *
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

#include <boost/bind.hpp>

#include <algorithm>

const double cruisespeed = 0.8;
const double avoidspeed = 0.3;
const double avoidturn = 0.8;
const double minfrontdistance = 1.2; // 0.6
const double stopdist = 0.8;
const int avoidduration = 50;

//void laserScanCallback(ros::Publisher& pub, tf::TransformListener &listener,
//		tf::StampedTransform &transform, geometry_msgs::Twist& cmd_msg,
//		int& avoidcount, const sensor_msgs::LaserScanConstPtr& scan_data) {
void laserScanCallback(ros::Publisher& pub, geometry_msgs::Twist& cmd_msg,
		int& avoidcount, const sensor_msgs::LaserScanConstPtr& scan_data) {

	const sensor_msgs::LaserScan::_ranges_type& scan = scan_data->ranges;
	uint32_t sample_count = scan.size();

	bool obstruction = false;
	bool stop = false;

	// find the closest distance to the left and right and check if
	// there's anything in front
	double minleft = 1e6;
	double minright = 1e6;

	for (uint32_t i = 0; i < sample_count; ++i) {
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
		--avoidcount;
	} else {
		avoidcount = 0;
		cmd_msg.linear.x = cruisespeed;
		cmd_msg.angular.z = 0.0;
	}
	// publish cmd_msgs on topic cmd_vel using publisher pub
	pub.publish(cmd_msg);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "wander_node");
	ROS_INFO("Starting laser obstacle avoidance behaviour");

	ros::NodeHandle my_node;
	geometry_msgs::Twist cmd;

	tf::StampedTransform transform;
	tf::TransformListener tf;
	tf::MessageFilter<sensor_msgs::LaserScan> * tf_filter;
	message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub;
	//ros::Subscriber laser_sub;

	int avoid_count = 0;

	ros::Publisher pub_vel = my_node.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	ROS_INFO_STREAM("Topic " << pub_vel.getTopic() << " advertised");

//	laser_sub = my_node.subscribe<sensor_msgs::LaserScan>
//	("scan", 1, boost::bind(&laserScanCallback, boost::ref(pub_vel), boost::ref(tf),
//			boost::ref(transform), boost::ref(cmd), boost::ref(avoid_count), _1));
	laser_sub.subscribe(my_node, "scan", 10);
	tf_filter = new tf::MessageFilter<sensor_msgs::LaserScan>(laser_sub, tf, "/base_link", 10);
	tf_filter->registerCallback(boost::bind(&laserScanCallback, boost::ref(pub_vel), boost::ref(cmd),
			boost::ref(avoid_count), _1));


	ROS_INFO_STREAM("Subscribed to topic " << laser_sub.getTopic());

	ros::spin();

	return 0;
}


