/*
 * Drawer.h
 *
 *  Created on: 7 сент. 2019 г.
 *      Author: Pavel
 */

#ifndef DRAWER_H_
#define DRAWER_H_

#include <EV3.h>
#include <memory>
#include <vector>

#include "Point.h"
#include "Process.h"

typedef std::shared_ptr<ev3::Motor> MotorPtr;
typedef std::shared_ptr<ev3::Sensor> SensorPtr;
typedef std::vector<Point> Polyline;

class Drawer {
public:
	Drawer(const MotorPtr &leftMotor, const MotorPtr &rightMotor, const MotorPtr &lifter, const SensorPtr &leftButton, const SensorPtr &rightButton);
	virtual ~Drawer();

	// calibrate (reset) drawer on start
	std::shared_ptr<ev3::Process> calibrate();
	// lift up the pen
	std::shared_ptr<ev3::Process> penUp();
	// put the pen down
	std::shared_ptr<ev3::Process> penDown();
	// move pen to specified position
	std::shared_ptr<ev3::Process> moveTo(const Point &position);
	// draw polylines in series
	std::shared_ptr<ev3::Process> drawLines(const std::vector<Polyline> &lines);

protected:
	enum class PenState {
		unknown, down, up
	};

	bool isCalibrated;
	PenState penState;
	Point penPosition;

	MotorPtr leftMotor;
	MotorPtr rightMotor;
	MotorPtr lifter;
	SensorPtr leftButton;
	SensorPtr rightButton;

	/**
	 *      .E   (pen is in coordinates E = (xE,yE))
	 *     / \
	 *    /   \
	 *   /     \
	 * C.       .D
	 *   \     /
	 *    \   /
	 *    A. .B
	 *   -------
	 *   [robot]
	 *   -------
	 */
	Point leftJoint; // A point
	Point rightJoint; // B point
	float arm1; // AC and BD distance
	float arm2; // CE and ED distance

	float getAlphaFromPosition(const Point &joint, const Point &target);
};

#endif /* DRAWER_H_ */
