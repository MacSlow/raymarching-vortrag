#include <cmath>

#include "vec3.h"

vec3::vec3 (float a) : _data {a, a, a} {}

vec3::vec3 (float x, float y, float z) : _data {x, y, z} {}

vec3::~vec3 () {}

vec3 vec3::operator+ (const vec3& a) const
{
	return vec3 (x() + a.x(), y() + a.y(), z() + a.z());
}

vec3& vec3::operator+= (const vec3& a)
{
	_data[X] += a.x();
	_data[Y] += a.y();
	_data[Z] += a.z();

	return *this;
}

vec3 vec3::operator- (const vec3& a) const
{
	return vec3 (x() - a.x(), y() - a.y(), z() - a.z());
}

vec3& vec3::operator-= (const vec3& a)
{
	_data[X] -= a.x();
	_data[Y] -= a.y();
	_data[Z] -= a.z();

	return *this;
}

vec3 vec3::operator* (float s) const
{
	return vec3 (s*x(), s*y(), s*z());
}

vec3& vec3::operator*= (float s)
{
	_data[X] *= s;
	_data[Y] *= s;
	_data[Z] *= s;

	return *this;
}

float dot (const vec3& a, const vec3& b)
{
	return a.x ()*b.x() + a.y()*b.y() + a.z()*b.z();
}

vec3 cross (const vec3& a, const vec3& b)
{
	return vec3 (a.y()*b.z() - a.z()*b.y(),
                 a.z()*b.x() - a.x()*b.z(),
                 a.x()*b.y() - a.y()*b.x());
}

vec3 normalize (const vec3& a)
{
	float f = 1.f / length (a);
	return vec3 (a.x()*f, a.y()*f, a.z()*f);
}

float length (const vec3& a)
{
	return sqrt (a.x()*a.x() + a.y()*a.y() + a.z()*a.z());
}

vec3 reflect (const vec3& i, const vec3& n)
{
	return i -  (n * dot(n, i) * 2.f);
}

vec3 abs (const vec3& v)
{
	float x = std::abs (v.x());
	float y = std::abs (v.y());
	float z = std::abs (v.z());
	return vec3 (x, y, z);
}

float vec3::x() const
{
	return _data[X];
}
float vec3::y() const
{
	return _data[Y];
}
float vec3::z() const
{
	return _data[Z];
}
