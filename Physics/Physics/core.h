#ifndef __CORE_H_INCLUDED__
#define __CORE_H_INCLUDED__


#include <windows.h>
#include <iostream>
#include "precision.h"

class Vector2
{
public:
	// variable
	real x;
	real y;

	// constructor
	Vector2();
	Vector2(real x, real y);

	// function
	void print() const;

	void invert();
	real magnitude() const;
	real squareMagnitude() const;
	void normalize();
	Vector2 unit() const;

	Vector2 operator*(real a) const;
	void scale(real a);
	Vector2 operator+(const Vector2& v) const;
	void add(const Vector2& v);
	Vector2 operator-(const Vector2& v) const;
	void minus(const Vector2& v);
	void addScaledVector(const Vector2& v, real a);
	
	Vector2 componentProduct(const Vector2 &v) const;
	void componentProductUpdate(const Vector2 &v);
	real dotProduct(const Vector2 &v) const;
	real operator*(const Vector2 &v) const;
	real crossProduct(const Vector2 &v) const;

	void rotate(real angle);
	void clear();
};


#endif // __CORE_H_INCLUDED__