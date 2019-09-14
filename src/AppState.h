/*
 * AppState.h
 *
 *  Created on: 15 сент. 2019 г.
 *      Author: Pavel
 */

#ifndef APPSTATE_H_
#define APPSTATE_H_

#include <memory>
#include <ev3.h>

class AppState {
public:
	static std::shared_ptr<AppState> shared;
	static void init(const std::shared_ptr<ev3::EV3> &ev3) {
		shared = std::shared_ptr<AppState>(new AppState(ev3));
	}

	const std::shared_ptr<ev3::EV3> ev3;

	virtual ~AppState();
protected:
	AppState(const std::shared_ptr<ev3::EV3> &ev3);
};

#endif /* APPSTATE_H_ */
