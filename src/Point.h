/*
 * Point.h
 *
 *  Created on: 7 сент. 2019 г.
 *      Author: Pavel
 */

#ifndef POINT_H_
#define POINT_H_

class Point {
public:
	float x;
	float y;
public:
	Point(float x = 0, float y = 0);

	Point& operator+=(const Point &p2);
	Point& operator-=(const Point &p2);
	Point& operator*=(float scale);
	Point& operator/=(float scale);

	float length() const;
};

Point operator+(const Point &p1, const Point &p2);
Point operator-(const Point &p1, const Point &p2);

Point operator*(const Point &p1, float scale);
Point operator/(const Point &p1, float scale);

#endif /* POINT_H_ */
