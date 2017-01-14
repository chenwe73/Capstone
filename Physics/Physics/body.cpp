#include "body.h"

RigidBody::RigidBody()
	: orientation(1, 0)
{
	inverseMass = 1;
	inverseMomentOfInertia = 1;
	linearDamping = (real)0.99;
	angularDamping = (real)0.99;
	calculateDerivedData();
}

RigidBody::RigidBody(const Vector2 &position, const Vector2 &orientation,
	real inverseMass, real inverseMomentOfInertia)
	: orientation(1, 0)
{
	this->position = position;
	if (orientation.magnitude() > 0)
		this->orientation = orientation;
	this->inverseMass = inverseMass;
	this->inverseMomentOfInertia = inverseMomentOfInertia;
	linearDamping = (real)0.99;
	angularDamping = (real)0.99;
	calculateDerivedData();
}

Vector2 RigidBody::getPosition() const
{
	return position;
}

Vector2 RigidBody::getOrientation() const
{
	return orientation;
}

Vector2 RigidBody::getVelocity() const
{
	return velocity;
}

real RigidBody::getAngularVelocity() const
{
	return angularVelocity;
}

Vector2 RigidBody::getAcceleration() const
{
	return acceleration;
}

Matrix3 RigidBody::getTransformMatrix() const
{
	return transformMatrix;
}

real RigidBody::getInverseMass() const
{
	return inverseMass;
}

real RigidBody::getMass() const
{
	if (inverseMass == 0)
		return 0;

	return 1.0f / inverseMass;
}

real RigidBody::getInverseMomentOfInertia() const
{
	return inverseMomentOfInertia;
}

void RigidBody::calculateDerivedData()
{
	orientation.normalize();
	transformMatrix.setOrientationAndPos(orientation, position);
}

void RigidBody::addForce(const Vector2 &force)
{
	forceAccum.add(force);
}

void RigidBody::integrate(real duration)
{
	// a = f / m
	Vector2 a = acceleration;
	a.addScaledVector(forceAccum, inverseMass);
	real angularAcceleration = torqueAccum * inverseMomentOfInertia;
	// v = v + a*t
	velocity.addScaledVector(a, duration);
	angularVelocity += angularAcceleration * duration;
	// v = v * d^t
	velocity.scale(real_pow(linearDamping, duration));
	angularVelocity *= real_pow(angularDamping, duration);
	// p = p + v*t     + (1/2 * a*t^2) ~ 0
	position.addScaledVector(velocity, duration);
	orientation.rotate(angularVelocity * duration);

	calculateDerivedData();
	clearAccumulators();
}

void RigidBody::clearAccumulators()
{
	forceAccum.clear();
	torqueAccum = 0;
}

Vector2 RigidBody::getPointInWorldSpace(const Vector2 &point)
{
	return (transformMatrix * point);
}

Vector2 RigidBody::getPointInLocalSpace(const Vector2 &point)
{
	return (transformMatrix.transformInverse(point));
}

void RigidBody::addForceAtBodyPoint(const Vector2 &force, const Vector2 &point)
{
	Vector2 world = transformMatrix * point;
	addForceAtPoint(force, world);
}

void RigidBody::addForceAtPoint(const Vector2 &force, const Vector2 &point)
{
	Vector2 arm = point - position;
	forceAccum.add(force);
	torqueAccum += arm.crossProduct(force);
}

void RigidBody::applyImpulseAtPoint(const Vector2& impulse, const Vector2 &point)
{
	// linear
	velocity.add(impulse *  inverseMass);
	// angular
	Vector2 arm = point - position;
	real angularImpulse = impulse.crossProduct(arm) * -1;
	real deltaAngularVelocity = angularImpulse * inverseMomentOfInertia;
	angularVelocity += deltaAngularVelocity;
}

bool RigidBody::isFiniteMass() const
{
	return (inverseMass > 0);
}

Vector2 RigidBody::getVelocityAtPoint(const Vector2 &point)
{
	// v = theta_dot.cross(r);
	Vector2 rotVelLocal = point.crossProduct(-point.magnitude() * angularVelocity);
	Vector2 rotVelWorld = transformMatrix.transformDirection(rotVelLocal);
	return velocity + rotVelWorld;
}

void RigidBody::move(const Vector2& displacement)
{
	position.add(displacement);
}

void RigidBody::rotate(real rotation)
{
	orientation.rotate(rotation);
}