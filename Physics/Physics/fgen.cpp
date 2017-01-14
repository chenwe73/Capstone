#include "fgen.h"

void ForceRegistry::add(RigidBody *body, ForceGenerator *fg)
{
	ForceRegistration registration;
	registration.body = body;
	registration.fg = fg;
	registrations.push_back(registration);
}

void ForceRegistry::updateForces(real duration)
{
	Registry::iterator i = registrations.begin();
	for (; i != registrations.end(); i++)
		i->fg->updateForce(i->body, duration);
}

Gravity::Gravity(const Vector2& gravity)
{
	this->gravity = gravity;
}

void Gravity::setGravity(const Vector2& gravity)
{
	this->gravity = gravity;
}

void Gravity::updateForce(RigidBody *body, real duration)
{
	if (!body->isFiniteMass())
		return;

	body->addForce(gravity * body->getMass());
}

Spring::Spring(const Vector2 &connectionPoint, RigidBody *other,
	const Vector2 &otherConnectionPoint,
	real springConstant, real restLength)
{
	this->connectionPoint = connectionPoint;
	this->other = other;
	this->otherConnectionPoint = otherConnectionPoint;
	this->springConstant = springConstant;
	this->restLength = restLength;
}

void Spring::updateForce(RigidBody *body, real duration)
{
	Vector2 connectionWorld = body->getPointInWorldSpace(connectionPoint);
	Vector2 otherConnectionWorld = other->getPointInWorldSpace(otherConnectionPoint);
	Vector2 v = connectionWorld - otherConnectionWorld;
	
	// F = -k * (l - l0)
	Vector2 force = v.unit() * (-(v.magnitude() - restLength) * springConstant);
	body->addForceAtPoint(force, connectionWorld);
}

Field::Field(const Vector2& source, real radius, real k, const Vector2& point)
{
	this->source = source;
	this->radius = radius;
	this->k = k;
	this->point = point;
}

void Field::updateForce(RigidBody *body, real duration)
{
	Vector2 v = body->getPointInWorldSpace(point) - source;
	real d = v.magnitude();

	if (d == 0)
		return;
	if (d <= radius)
	{
		Vector2 force = v.unit() * (k / (d*d));
		body->addForceAtBodyPoint(force, point);
	}
}

void Field::setSource(const Vector2& s)
{
	source = s;
}

Aero::Aero(const Matrix2 &tensor, const Vector2 &position,
	const Vector2 *windSpeed)
{
	this->tensor = tensor;
	this->position = position;
	this->windSpeed = windSpeed;
}

void Aero::updateForce(RigidBody *body, real duration)
{
	Vector2 velocityWorld = body->getVelocity() + *windSpeed;
	Vector2 velocityBody = body->getTransformMatrix().transformInverseDirection(velocityWorld);
	Vector2 forceBody = tensor * velocityBody;
	Vector2 forceWorld = body->getTransformMatrix().transformDirection(forceBody);
	body->addForceAtBodyPoint(forceWorld, position);
}

Buoyancy::Buoyancy(real maxDepth, real volume, real waterHeight,
	real liquidDensity, Vector2 centerOfBuoyancy)
{
	this->maxDepth = maxDepth;
	this->volume = volume;
	this->waterHeight = waterHeight;
	this->liquidDensity = liquidDensity;
	this->centerOfBuoyancy = centerOfBuoyancy;
}


void Buoyancy::updateForce(RigidBody *body, real duration)
{
	Vector2 point = body->getPointInWorldSpace(centerOfBuoyancy);
	real depth = point.y;
	Vector2 force(0, 0);

	if (depth >= waterHeight + maxDepth) // out of water
		return;

	if (depth <= waterHeight - maxDepth) // fully submerged
		force.y = liquidDensity * volume;
	else // partially submerged
		force.y = liquidDensity * volume
		* (maxDepth - (depth - waterHeight)) / (2 * maxDepth);

	body->addForceAtPoint(force, point);
}