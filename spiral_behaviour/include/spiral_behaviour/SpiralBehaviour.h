/*
 * SpiralBehaviour.h
 *
 *  Created on: 4 Feb 2013
 *      Author: Murilo F. M.
 *      Email: muhrix@gmail.com
 *
 *	http://en.wikipedia.org/wiki/Archimedean_spiral
 *	The Archimedean spiral has the property that any ray from the origin
 *	intersects successive turnings of the spiral in points with a constant
 *	separation distance (equal to 2πb if θ is measured in radians), hence the
 *	name "arithmetic spiral"
 *
 */

#ifndef SPIRALBEHAVIOUR_H_
#define SPIRALBEHAVIOUR_H_

#include <cmath>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_listener.h"

class SpiralBehaviour {
public:
	SpiralBehaviour();                             // default constructor
	SpiralBehaviour(double a, double b, double v); // initialisation constructor
	SpiralBehaviour(const SpiralBehaviour &);      // copy constructor
	~SpiralBehaviour();
	const SpiralBehaviour &operator= (const SpiralBehaviour &);  // assignment operator

	// getters
	double GetParamA(void);
	double GetParamB(void);
	double GetParamR(void);

	// setters
	void SetParamA(double a);
	void SetParamB(double b);
	void SetParamV(double v);

	// public member functions
	geometry_msgs::Twist CalculateVels(const tf::StampedTransform &);

private:
	double a_;
	double b_;
	double r_;
	double theta_;
	double v_;
	double w_;
	double turns_;
	int prevThetaSign_;
	geometry_msgs::Twist spiralCmd_;

	// private member functions
	double NormaliseTheta(double theta);
};

#endif /* SPIRALBEHAVIOUR_H_ */
