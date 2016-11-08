#include "core.h"

Vector2::Vector2()
{
	x = 0;
	y = 0;
}

Vector2::Vector2(real x, real y)
{
	Vector2::x = x;
	Vector2::y = y;
}

void Vector2::print() const
{
	std::cout << '<' << x << ", " << y << '>';
}

void Vector2::invert()
{
	x = -x;
	y = -y;
}

real Vector2::magnitude() const
{
	return real_sqrt(x*x + y*y);
}

real Vector2::squareMagnitude() const
{
	return x*x + y*y;
}

void Vector2::normalize()
{
	real m = magnitude();
	if (m > 0)
		scale((real)1.0 / m);
}

Vector2 Vector2::unit() const
{
	real m = magnitude();
	if (m <= 0)
		return Vector2(0, 0);
	return Vector2(x / m, y / m);
}

Vector2 Vector2::operator*(real a) const
{
	return Vector2(x * a, y * a);
}

void Vector2::scale(real a)
{
	x = x * a;
	y = y * a;
}

Vector2 Vector2::operator+(const Vector2& v) const
{
	return Vector2(x + v.x, y + v.y);
}

void Vector2::add(const Vector2& v)
{
	x = x + v.x;
	y = y + v.y;
}

Vector2 Vector2::operator-(const Vector2& v) const
{
	return Vector2(x - v.x, y - v.y);
}

void Vector2::minus(const Vector2& v)
{
	x = x - v.x;
	y = y - v.y;
}

void Vector2::addScaledVector(const Vector2& v, real a)
{
	x = x + v.x * a;
	y = y + v.y * a;
}

Vector2 Vector2::componentProduct(const Vector2 &v) const
{
	return Vector2(x * v.x, y * v.y);
}

void Vector2::componentProductUpdate(const Vector2 &v)
{
	x = x * v.x;
	y = y * v.y;
}

real Vector2::dotProduct(const Vector2 &v) const
{
	return x * v.x + y * v.y;
}

real Vector2::operator*(const Vector2 &v) const
{
	return x * v.x + y * v.y;
}

real Vector2::crossProduct(const Vector2 &v) const
{
	return x * v.y - y * v.x;
}

void Vector2::rotate(real angle)
{
	real x2 = x * cos(angle) - y * sin(angle);
	real y2 = x * sin(angle) + y * cos(angle);
	x = x2;
	y = y2;
}

void Vector2::clear()
{
	x = 0;
	y = 0;
}

