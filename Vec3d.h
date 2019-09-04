#pragma once
#ifndef __VEC3D_H__
#define __VEC3D_H__
#include <cmath>
#include <string>
#include <iostream>
#include <iomanip>

// a minimal vector class of 3 doubles and overloaded math operators
/*
*��С���������������ѧ��������
*/
class Vec3 {
public:

	double f[3];

	Vec3(double x, double y, double z) {
		f[0] = x;
		f[1] = y;
		f[2] = z;
	}

	Vec3() {}

	double length() {
		return sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
	}

	Vec3 normalized() {
		double l = length();

		return Vec3(f[0] / l, f[1] / l, f[2] / l);
	}

	void operator+=(const Vec3& v) {
		f[0] += v.f[0];
		f[1] += v.f[1];
		f[2] += v.f[2];
	}

	Vec3 operator/(const double& a) {
		return Vec3(f[0] / a, f[1] / a, f[2] / a);
	}

	Vec3 operator-(const Vec3& v) {
		return Vec3(f[0] - v.f[0], f[1] - v.f[1], f[2] - v.f[2]);
	}

	Vec3 operator+(const Vec3& v) {
		return Vec3(f[0] + v.f[0], f[1] + v.f[1], f[2] + v.f[2]);
	}

	Vec3 operator*(const double& a) {
		return Vec3(f[0] * a, f[1] * a, f[2] * a);
	}

	Vec3 operator-() {
		return Vec3(-f[0], -f[1], -f[2]);
	}

	Vec3 cross(const Vec3& v) {
		return Vec3(
			f[1] * v.f[2] - f[2] * v.f[1],
			f[2] * v.f[0] - f[0] * v.f[2],
			f[0] * v.f[1] - f[1] * v.f[0]
		);
	}

	double dot(const Vec3& v) {
		return f[0] * v.f[0] + f[1] * v.f[1] + f[2] * v.f[2];
	}
};

#endif//__VEC3D_H__