/*
 * spiral_behaviour.cpp
 *
 *  Created on: 1 Feb 2013
 *      Author: Murilo F. M.
 *      Email: muhrix@gmail.com
 *
 *  Updated on: 31 Jan 2014
 */

#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "tf/transform_listener.h"

#include "spiral_behaviour/SetParams.h"
#include "spiral_behaviour/SetLinearVel.h"
#include "spiral_behaviour/GetParams.h"
#include "spiral_behaviour/GetRadius.h"

#include <boost/bind.hpp>

#include "spiral_behaviour/SpiralBehaviour.h"

//void robotPoseCallback(ros::NodeHandle &node_handle,
//		const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose) {

void RobotPoseCallback(ros::Publisher &pub, SpiralBehaviour &sb, tf::TransformListener &listener,
		tf::StampedTransform &transform,
		const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose) {

	try {
		listener.lookupTransform("/odom", "/base_link",
    		ros::Time(0), transform);

		geometry_msgs::Twist spiralCmd;
		spiralCmd = sb.CalculateVels(transform);

		pub.publish(spiralCmd);
	}
	catch (tf::TransformException &ex) {
		ROS_ERROR("%s",ex.what());
	}
}

bool SetParams(SpiralBehaviour &sb, spiral_behaviour::SetParams::Request &req,
		spiral_behaviour::SetParams::Response &res) {

	sb.SetParamA(req.a);
	sb.SetParamB(req.b);
	return true;
}

bool SetLinearVel(SpiralBehaviour &sb, spiral_behaviour::SetLinearVel::Request &req,
		spiral_behaviour::SetLinearVel::Response &res) {

	sb.SetParamV(req.v);
	return true;
}

bool GetParams(SpiralBehaviour &sb, spiral_behaviour::GetParams::Request &req,
		spiral_behaviour::GetParams::Response &res) {

	res.a = sb.GetParamA();
	res.b = sb.GetParamB();
	return true;
}

bool GetRadius(SpiralBehaviour &sb, spiral_behaviour::GetRadiusRequest &req,
		spiral_behaviour::GetRadiusResponse &res) {

	res.r = sb.GetParamR();
	return true;
}

int main(int argc, char* argv[]) {
	SpiralBehaviour sb(0.0, 0.2, 1.0);

	ros::init(argc, argv, "spiral_node");
	ROS_INFO("Starting Archimedean spiral behaviour");

	ros::NodeHandle myNode;

	ros::Publisher spiral_pub = myNode.advertise<geometry_msgs::Twist>(
			"/cmd_vel", 1);
	ROS_INFO_STREAM("Topic " << spiral_pub.getTopic() << " advertised");

	tf::TransformListener listener;
	tf::StampedTransform transform;

	//ros::SubscribeOptions subOpts;
	//subOpts.topic = "/robot_pose_ekf/odom_combined";
	//subOpts.queue_size = 1;


	ros::Subscriber spiral_sub = myNode.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
			"/robot_pose_ekf/odom", 1,
			boost::bind(&RobotPoseCallback, boost::ref(spiral_pub), boost::ref(sb),
			boost::ref(listener), boost::ref(transform), _1));

	ROS_INFO_STREAM("Subscribed to topic " << spiral_sub.getTopic());

	ros::ServiceServer set_params_srv = myNode.advertiseService<spiral_behaviour::SetParamsRequest,
			spiral_behaviour::SetParamsResponse>(
					"set_params", boost::bind(&SetParams, boost::ref(sb), _1, _2));

	ROS_INFO_STREAM("Service " << set_params_srv.getService() << " advertised");

	ros::ServiceServer set_lin_vel_srv = myNode.advertiseService<spiral_behaviour::SetLinearVelRequest,
			spiral_behaviour::SetLinearVelResponse>(
					"set_linear_velocity", boost::bind(&SetLinearVel, boost::ref(sb), _1, _2));

	ROS_INFO_STREAM("Service " << set_lin_vel_srv.getService() << " advertised");

	ros::ServiceServer get_params_srv = myNode.advertiseService<spiral_behaviour::GetParamsRequest,
			spiral_behaviour::GetParamsResponse>(
					"get_params", boost::bind(&GetParams, boost::ref(sb), _1, _2));

	ROS_INFO_STREAM("Service " << get_params_srv.getService() << " advertised");

	ros::ServiceServer get_radius_srv = myNode.advertiseService<spiral_behaviour::GetRadiusRequest,
			spiral_behaviour::GetRadiusResponse>(
					"get_radius", boost::bind(&GetRadius, boost::ref(sb), _1, _2));

	ROS_INFO_STREAM("Service " << get_radius_srv.getService() << " advertised");

	//ros::Rate loop_rate(10);

	ros::spin();

	//std::cout << "\nShutting down Archimedean spiral behaviour." << std::endl;

	return 0;
}


