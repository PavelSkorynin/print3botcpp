/*
 * Drawer.cpp
 *
 *  Created on: 7 сент. 2019 г.
 *      Author: Pavel
 */

#include "Drawer.h"
#include "ProcessSequence.h"
#include "ProcessGroup.h"
#include "PD.h"

#include "AppState.h"

#include <math.h>

#define MAX_POWER 20

Drawer::Drawer(const MotorPtr &leftMotor, const MotorPtr &rightMotor, const MotorPtr &lifter, const SensorPtr &leftButton, const SensorPtr &rightButton)
	: isCalibrated(false)
	, penState(PenState::unknown)
	, leftMotor(leftMotor)
	, rightMotor(rightMotor)
	, lifter(lifter)
	, leftButton(leftButton)
	, rightButton(rightButton)

	, leftJoint(-3.5f, -6.0f)
	, rightJoint(3.5f, -6.0f)
	, arm1(11.0f)
	, arm2(14.0f)
{
	leftMotor->setMaxAccelleration(100000);
	rightMotor->setMaxAccelleration(100000);
}

Drawer::~Drawer() {
}

// calibrate (reset) drawer on start
std::shared_ptr<ev3::Process> Drawer::calibrate() {
	auto sequence = std::make_shared<ev3::ProcessSequence>();
	if (leftButton->getValue().getValue() != 0 || rightButton->getValue().getValue() != 0) {
		auto group = std::make_shared<ev3::ProcessGroup>();
		if (leftButton->getValue().getValue() != 0) {
			auto process = std::make_shared<ev3::LambdaProcess>([&](float) -> bool {
				if (leftButton->getValue().getValue() == 0) {
					return false;
				}
				leftMotor->setPower(MAX_POWER);
				return true;
			}, [&](float) {
				leftMotor->setPower(0);
			});
			group->addProcess(std::dynamic_pointer_cast<ev3::Process>(process));
		}
		if (rightButton->getValue().getValue() != 0) {
			auto process = std::make_shared<ev3::LambdaProcess>([&](float) -> bool {
				if (rightButton->getValue().getValue() == 0) {
					return false;
				}
				rightMotor->setPower(MAX_POWER);
				return true;
			}, [&](float) {
				rightMotor->setPower(0);
			});
			group->addProcess(std::dynamic_pointer_cast<ev3::Process>(process));
		}
		sequence->addProcess(std::dynamic_pointer_cast<ev3::Process>(group));
	}

	auto process = std::make_shared<ev3::LambdaProcess>([&](float timestamp) -> bool {
		if (leftButton->getValue().getValue() != 0 && rightButton->getValue().getValue() != 0) {
			return false;
		}
		if (leftButton->getValue().getValue() == 0) {
			leftMotor->setPower(-MAX_POWER);
		}
		if (rightButton->getValue().getValue() == 0) {
			rightMotor->setPower(-MAX_POWER);
		}
		return true;
	}, [&](float) {
		leftMotor->setPower(0);
		rightMotor->setPower(0);
		leftMotor->resetEncoder();
		rightMotor->resetEncoder();
		isCalibrated = true;
		throw 100;
	});
	sequence->addProcess(std::dynamic_pointer_cast<ev3::Process>(process));

	return std::dynamic_pointer_cast<ev3::Process>(sequence);
}

// lift up the pen
std::shared_ptr<ev3::Process> Drawer::penUp() {
	auto process = std::make_shared<ev3::TimeProcess>([&](float) {
		lifter->setPower(20);
	}, [&](float) {
		lifter->setPower(0);
	}, 0.1f);

	return std::dynamic_pointer_cast<ev3::Process>(process);
}

// put the pen down
std::shared_ptr<ev3::Process> Drawer::penDown() {
	auto process = std::make_shared<ev3::TimeProcess>([&](float) {
		lifter->setPower(-20);
	}, [&](float) {
		lifter->setPower(0);
	}, 0.1f);

	return std::dynamic_pointer_cast<ev3::Process>(process);
}

// move pen to specified position
std::shared_ptr<ev3::Process> Drawer::moveTo(const Point &position) {

	float angleLeft = getAlphaFromPosition(leftJoint, position) * 180 / M_PI;
	float angleRight = getAlphaFromPosition(rightJoint, position) * 180 / M_PI;

	float targetEncoderLeft = ((angleLeft-14.) * 2970. / (90.-14.));
	float targetEncoderRight = ((angleRight-14.) * 2970. / (90.-14.));

	auto pdLeft = std::shared_ptr<ev3::PD>(new ev3::PD());
	pdLeft->setError(ev3::WireF(targetEncoderLeft) - ev3::WireF(leftMotor->getEncoder()));

	auto pdRight = std::shared_ptr<ev3::PD>(new ev3::PD());
	pdRight->setError(ev3::WireF(targetEncoderRight) - ev3::WireF(rightMotor->getEncoder()));

	auto process = std::make_shared<ev3::LambdaProcess>([=](float timestamp) -> bool {
		pdLeft->update(timestamp);
		pdRight->update(timestamp);

		leftMotor->setPower(fmaxf(fminf(pdLeft->getPower().getValue(), 100.0f), -100.0f));
		rightMotor->setPower(fmaxf(fminf(pdRight->getPower().getValue(), 100.0f), -100.0f));

		if (fabsf(targetEncoderLeft - leftMotor->getEncoder().getValue()) < 5
				&& fabsf(targetEncoderRight - rightMotor->getEncoder().getValue()) < 5) {
			return false;
		}

		return true;
	}, [&](float) {
		leftMotor->setPower(0);
		rightMotor->setPower(0);
	});

	return std::dynamic_pointer_cast<ev3::Process>(process);
}

// draw polylines in series
std::shared_ptr<ev3::Process> Drawer::drawLines(const std::vector<Polyline> &lines) {
	auto sequence = std::make_shared<ev3::ProcessSequence>();
	sequence->addProcess(penUp());
	if (!isCalibrated) {
		sequence->addProcess(calibrate());
	}
	for (auto &polyline : lines) {
		if (polyline.empty()) {
			continue;
		}
		auto lineIt = polyline.begin();
		sequence->addProcess(moveTo(*lineIt));
		sequence->addProcess(penDown());
		lineIt++;
		for (; lineIt != polyline.end(); ++lineIt) {
			sequence->addProcess(moveTo(*lineIt));
		}
		sequence->addProcess(penUp());
	}
	return std::dynamic_pointer_cast<ev3::Process>(sequence);
}

float Drawer::getAlphaFromPosition(const Point &joint, const Point &target) {
	float d = (target - joint).length();
	float d1 = (d * d + arm1 * arm1 - arm2 * arm2) / 2 / d;
	float h = sqrtf(arm1 * arm1 - d1 * d1);
	Point P2 = joint + (target - joint) * (d * d + arm1 * arm1 - arm2 * arm2) / 2 / d / d;
	float x3 = P2.x + h * (target.y - joint.y) / d;
	float y3 = P2.y - h * (target.x - joint.x) / d;
	float dx = x3 - joint.x;
	float dy = y3 - joint.y;
	return atan2f(dy, dx);
}

