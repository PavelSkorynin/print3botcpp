/*
 * AppState.cpp
 *
 *  Created on: 15 сент. 2019 г.
 *      Author: Pavel
 */

#include "AppState.h"

std::shared_ptr<AppState> AppState::shared = std::shared_ptr<AppState>();

AppState::AppState(const std::shared_ptr<ev3::EV3> &ev3)
: ev3(ev3)
{
}

AppState::~AppState() {
}

