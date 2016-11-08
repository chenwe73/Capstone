#include "particle.h"

Particle::Particle()
{
	position = Vector2(0, 0);
	velocity = Vector2(0, 0);
	acceleration = Vector2(0, 0);
	damping = (real)0.99;
	inverseMass = 1;
	forceAccum = Vector2(0, 0);
}

Particle::Particle(Vector2 p, Vector2 v, Vector2 a, real d, real im, Vector2 fa)
{
	position = p;
	velocity = v;
	acceleration = a;
	damping = d;
	inverseMass = im;
	forceAccum = fa;
}


Vector2 Particle::getPosition() const
{
	return position;
}

Vector2 Particle::getVelocity() const
{
	return velocity;
}

void Particle::integrate(real duration)
{
	// p = p + v*t
	position.addScaledVector(velocity, duration);

	// F = m*a
	Vector2 resultingAcc = acceleration;
	resultingAcc.addScaledVector(forceAccum, inverseMass);

	// v = v + a*t
	velocity.addScaledVector(resultingAcc, duration);

	// v = v * d^t
	velocity.scale(real_pow(damping, duration));

	// clear forces
	clearAcumulator();
}

void Particle::clearAcumulator()
{
	forceAccum.clear();
}

void Particle::addForce(const Vector2& force)
{
	forceAccum.add(force);
}

bool Particle::hasFiniteMass() const
{
	return (inverseMass != 0);
}

real Particle::getMass() const
{
	return 1 / inverseMass;
}

Vector2 Particle::getvelocity() const
{
	return velocity;
}
