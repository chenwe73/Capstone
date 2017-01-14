#ifndef __FGEN_H_INCLUDED__
#define __FGEN_H_INCLUDED__


#include <vector>

#include "precision.h"
#include "core.h"
#include "body.h"

class ForceGenerator
{
public:
	virtual void updateForce(RigidBody *body, real duration) = 0;
};

class ForceRegistry
{
protected:
	struct ForceRegistration
	{
		RigidBody *body;
		ForceGenerator *fg;
	};

	typedef std::vector<ForceRegistration> Registry;
	Registry registrations;

public:
	void add(RigidBody *body, ForceGenerator *fg);
	void remove(RigidBody *body, ForceGenerator *fg);
	void clear();
	// calls updateForce for every registored ForceGenerator
	void updateForces(real duration);
};

class Gravity : public ForceGenerator
{
private:
	Vector2 gravity;

public:
	Gravity(const Vector2& gravity);
	void setGravity(const Vector2& gravity);
	virtual void updateForce(RigidBody *body, real duration);
};

class Spring : public ForceGenerator
{
public:
	Vector2 connectionPoint;
	RigidBody *other;
	Vector2 otherConnectionPoint;
	real springConstant;
	real restLength;

public:
	Spring(const Vector2 &connectionPoint, RigidBody *other,
		const Vector2 &otherConnectionPoint, 
		real springConstant, real restLength);
	virtual void updateForce(RigidBody *body, real duration);
};

class Field : public ForceGenerator
{
private:
	Vector2 source;
	real radius;
	real k;
	Vector2 point;

public:
	Field(const Vector2& source, real radius, real k, const Vector2& point);
	virtual void updateForce(RigidBody *body, real duration);
	void setSource(const Vector2& s);
};

class Aero : public ForceGenerator
{
private:
	Matrix2 tensor;
	Vector2 position;
	const Vector2* windSpeed;

public:
	Aero(const Matrix2 &tensor, const Vector2 &position, 
		const Vector2 *windSpeed);
	virtual void updateForce(RigidBody *body, real duration);
};

class Buoyancy : public ForceGenerator
{
private:
	real maxDepth;
	real volume;
	real waterHeight;
	real liquidDensity;
	Vector2 centerOfBuoyancy;

public:
	Buoyancy(real maxDepth, real volume, real waterHeight, 
		real liquidDensity, Vector2 centerOfBuoyancy);
	virtual void updateForce(RigidBody *body, real duration);
};


#endif // __FGEN_H_INCLUDED__