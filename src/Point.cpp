/*
 * Point.cpp
 *
 *  Created on: 7 сент. 2019 г.
 *      Author: Pavel
 */

#include "Point.h"
#include <math.h>

Point::Point(float x, float y) : x(x), y(y) {

}

Point& Point::operator+=(const Point &p2) {
	x += p2.x;
	y += p2.y;
	return *this;
}

Point& Point::operator-=(const Point &p2) {
	x -= p2.x;
	y -= p2.y;
	return *this;
}

Point& Point::operator*=(float scale) {
	x *= scale;
	y *= scale;
	return *this;
}

Point& Point::operator/=(float scale) {
	x /= scale;
	y /= scale;
	return *this;
}

float Point::length() const {
	return sqrtf(x * x + y * y);
}

Point operator+(const Point &p1, const Point &p2) {
	return Point(p1.x + p2.x, p1.y + p2.y);
}

Point operator-(const Point &p1, const Point &p2) {
	return Point(p1.x - p2.x, p1.y - p2.y);
}

Point operator*(const Point &p1, float scale) {
	return Point(p1.x * scale, p1.y * scale);
}

Point operator/(const Point &p1, float scale) {
	return Point(p1.x / scale, p1.y / scale);
}
