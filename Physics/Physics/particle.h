#ifndef __PARTICLE_H_INCLUDED__
#define __PARTICLE_H_INCLUDED__


#include "core.h"

class Particle
{
private:
	Vector2 position;
	Vector2 velocity;
	Vector2 acceleration;
	real damping; // 0 to 1
	real inverseMass; // 1/m
	Vector2 forceAccum;

public:
	Particle();
	Particle(Vector2 p, Vector2 v, Vector2 a, real d, real im, Vector2 fa);

	Vector2 getPosition() const;
	Vector2 getVelocity() const;

	void integrate(real duration);
	void clearAcumulator();
	void addForce(const Vector2& force);

	bool hasFiniteMass() const;
	real getMass() const;
	Vector2 getvelocity() const;
};


#endif // __PARTICLE_H_INCLUDED__