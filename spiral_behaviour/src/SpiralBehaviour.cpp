/*
 * SpiralBehaviour.cpp
 *
 *  Created on: 4 Feb 2013
 *      Author: Murilo F. M.
 *      Email: muhrix@gmail.com
 *
 * 	http://en.wikipedia.org/wiki/Archimedean_spiral
 *	The Archimedean spiral has the property that any ray from the origin
 *	intersects successive turnings of the spiral in points with a constant
 *	separation distance (equal to 2πb if θ is measured in radians), hence the
 *	name "arithmetic spiral"
 *
 */

#include "spiral_behaviour/SpiralBehaviour.h"

// default constructor
SpiralBehaviour::SpiralBehaviour():a_(0.0), b_(0.0), r_(0.0), theta_(0.0), v_(0.0), w_(0.0),
		turns_(0.0), prevThetaSign_(1) {

}

// initialisation constructor
SpiralBehaviour::SpiralBehaviour(double a, double b, double v):a_(a), b_(b), r_(0.0),
		theta_(0.0), v_(v), w_(0.0), turns_(0.0), prevThetaSign_(1) {

}

// copy constructor
SpiralBehaviour::SpiralBehaviour(const SpiralBehaviour & sb): a_(sb.a_), b_(sb.b_), r_(sb.r_),
		theta_(sb.theta_), v_(sb.v_), w_(sb.w_), turns_(sb.turns_),
		prevThetaSign_(sb.prevThetaSign_) {

	this->spiralCmd_.linear.x = sb.spiralCmd_.linear.x;
	this->spiralCmd_.linear.y = sb.spiralCmd_.linear.y;
	this->spiralCmd_.linear.z = sb.spiralCmd_.linear.z;

	this->spiralCmd_.angular.x = sb.spiralCmd_.angular.x;
	this->spiralCmd_.angular.y = sb.spiralCmd_.angular.y;
	this->spiralCmd_.angular.z = sb.spiralCmd_.angular.z;
}

// destructor
SpiralBehaviour::~SpiralBehaviour() {

}

// assignment operator overloading
const SpiralBehaviour &SpiralBehaviour::operator =(const SpiralBehaviour & sb) {
	if (&sb == this) { // checking for self-assignment
		return sb;
	}
	//else {
		this->a_ = sb.a_;
		this->b_ = sb.b_;
		this->r_ = sb.r_;
		this->theta_ = sb.theta_;
		this->v_ = sb.v_;
		this->w_ = sb.w_;
		this->turns_ = sb.turns_;
		this->prevThetaSign_ = sb.prevThetaSign_;

		this->spiralCmd_.linear.x = sb.spiralCmd_.linear.x;
		this->spiralCmd_.linear.y = sb.spiralCmd_.linear.y;
		this->spiralCmd_.linear.z = sb.spiralCmd_.linear.z;

		this->spiralCmd_.angular.x = sb.spiralCmd_.angular.x;
		this->spiralCmd_.angular.y = sb.spiralCmd_.angular.y;
		this->spiralCmd_.angular.z = sb.spiralCmd_.angular.z;

		return *this;
	//}
}

// private member functions
double SpiralBehaviour::NormaliseTheta(double theta) {
	if (theta < 0.0) {
		return (theta + 2.0*M_PI);
	}
	//else {
		return theta;
	//}
}

// public member functions
geometry_msgs::Twist SpiralBehaviour::CalculateVels(const tf::StampedTransform& transform) {
	double yaw = tf::getYaw(transform.getRotation());
	if (prevThetaSign_ == -1 && yaw >= 0) {
		++turns_;

		if (turns_ == 1)
			ROS_INFO_STREAM("Completed " << turns_ << "turn");
		else
			ROS_INFO_STREAM("Completed " << turns_ << "turns");
	}
	if (yaw < 0) {
		prevThetaSign_ = -1;
	}
	else {
		prevThetaSign_ = 1;
	}

	// orientation θ(t) is in radians already
	theta_ = this->NormaliseTheta(yaw);

	// calculate r(t)
	r_ = a_ + b_*(theta_ + turns_*2.0*M_PI);

	// It is necessary to calculate v: linear and w: angular velocities
	// v = constant velocity and v = w*r
	// w(t) = v(t) / r(t) => w(t) = v / r(t)
	// v = linear speed in X axis (robot's coordinate frame)
	// w = angular spped in Z axis (orthogonal to XY plane in robot's coordinate frame)

	// calculate w(t)
	w_ = v_ / r_;

	// set values to Twist variable
	spiralCmd_.linear.x = v_;
	spiralCmd_.angular.z = w_;

	return this->spiralCmd_;
}
// getters
double SpiralBehaviour::GetParamA() {
	return this->a_;
}

double SpiralBehaviour::GetParamB() {
	return this->b_;
}

double SpiralBehaviour::GetParamR() {
	return this->r_;
}

// setters
void SpiralBehaviour::SetParamA(double a) {
	this->a_ = a;
}

void SpiralBehaviour::SetParamB(double b) {
	this->b_ = b;
}

void SpiralBehaviour::SetParamV(double v) {
	this->v_ = v;
}


