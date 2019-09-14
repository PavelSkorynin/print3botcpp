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

	try {

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

	ev3->lcdPrintf(ev3::EV3::Color::BLACK, "start\n");
	ev3->wait(1);

	ev3->lcdPrintf(ev3::EV3::Color::BLACK, "init drawer\n");
	Drawer drawer(leftMotor, rightMotor, lifter, leftButton, rightButton);

	// read input
	ev3->lcdPrintf(ev3::EV3::Color::BLACK, "run calibration\n");

	ev3->wait(1);
	// calibrate drawer
	ev3->runProcess(drawer.calibrate());
	ev3->lcdPrintf(ev3::EV3::Color::BLACK, "done\n");
	ev3->wait(2);

	// draw
	}
	catch (std::exception& e)
	{
		ev3->lcdPrintf(ev3::EV3::Color::BLACK, "exception: %s\n", e.what());
		ev3->wait(2);
	}
	catch (int e)
	{
		ev3->lcdPrintf(ev3::EV3::Color::BLACK, "exception: %d\n", e);
		ev3->wait(2);
	}
	catch (...) {
		ev3->lcdPrintf(ev3::EV3::Color::BLACK, "unknown exception\n");
		ev3->wait(2);
	}

	ev3.reset();
	return 0;
}
