//============================================================================
// Name        : PortraitBot.cpp
// Author      : $(author)
// Version     :
// Copyright   : $(copyright)
// Description : Modern EV3 Hello World in C++
//============================================================================

#include <ev3.h>
#include <string>
#include <exception>

#include "Drawer.h"
#include "AppState.h"

int main()
{
	auto ev3 = std::make_shared<ev3::EV3>();
	AppState::init(ev3);

	// init devices
	auto leftMotor = ev3->getMotor(ev3::Motor::Port::A);
	auto rightMotor = ev3->getMotor(ev3::Motor::Port::B);
	auto lifter = ev3->getMotor(ev3::Motor::Port::C);

	auto leftButton = ev3->getSensor(ev3::Sensor::Port::P4, ev3::Sensor::Mode::TOUCH);
	auto rightButton = ev3->getSensor(ev3::Sensor::Port::P1, ev3::Sensor::Mode::TOUCH);

	leftMotor->setPower(0);
	rightMotor->setPower(0);
	lifter->setPower(0);

	leftMotor->setDirection(ev3::Motor::Direction::FORWARD);
	rightMotor->setDirection(ev3::Motor::Direction::BACKWARD);
	lifter->setDirection(ev3::Motor::Direction::FORWARD);

	Drawer drawer(leftMotor, rightMotor, lifter, leftButton, rightButton);

	// read input
	ev3->lcdPrintf(ev3::EV3::Color::BLACK, "run calibration\n");

	// calibrate drawer
	ev3->runProcess(drawer.calibrate());
	ev3->lcdPrintf(ev3::EV3::Color::BLACK, "calibration done\n");
	ev3->runProcess(drawer.penUp());

	// draw triangle
	auto polyline = std::vector<Point>();
	polyline.push_back(Point(0, 0));
	polyline.push_back(Point(10, 10));
	polyline.push_back(Point(-10, 10));
	polyline.push_back(Point(0, 0));

	auto lines = std::vector<std::vector<Point>>();
	lines.push_back(polyline);
	ev3->runProcess(drawer.drawLines(lines));
	ev3->wait(5);

	// draw
	ev3.reset();
	return 0;
}
