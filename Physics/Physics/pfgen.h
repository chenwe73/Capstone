#ifndef __PFGEN_H_INCLUDED__
#define __PFGEN_H_INCLUDED__


#include <vector>
#include "particle.h"
#include "core.h"

class ParticleForceGenerator
{
public:
	virtual void updateForce(Particle *particle, real duration) = 0;
};

class ParticleForceRegistry
{
protected:
	struct ParticleForceRegistration
	{
		Particle *particle;
		ParticleForceGenerator *fg;
	};

	typedef std::vector<ParticleForceRegistration> Registry;
	Registry registrations;

public:
	void add(Particle *particle, ParticleForceGenerator *fg);
	//void remove(Particle *particle, ParticleForceGenerator *fg);
	//void clear();
	void updateForces(real duration);
};

class ParticleGravity : public ParticleForceGenerator
{
private:
	Vector2 gravity;

public:
	ParticleGravity(const Vector2& g);
	virtual void updateForce(Particle *particle, real duration);
};

class ParticleDrag : public ParticleForceGenerator
{
private:
	real k1;
	real k2;

public:
	ParticleDrag(real k1, real k2);
	virtual void updateForce(Particle *particle, real duration);
};


class ParticleField : public ParticleForceGenerator
{
private:
	Vector2 source;
	real radius;
	real k;

public:
	ParticleField(const Vector2& s, real r, real k);
	virtual void updateForce(Particle *particle, real duration);
	void setSource(const Vector2& s);
};

class ParticleSpring : public ParticleForceGenerator
{
private:
	Particle *other;
	real springConstant;
	real restLength;

public:
	ParticleSpring(Particle *o, real sc, real rl);
	virtual void updateForce(Particle *particle, real duration);
};

class ParticleAnchoredSpring : public ParticleForceGenerator
{
private:
	Vector2 *anchor;
	real springConstant;
	real restLength;

public:
	ParticleAnchoredSpring(Vector2 *a, real sc, real rl);
	virtual void updateForce(Particle *particle, real duration);
};


#endif // __PFGEN_H_INCLUDED__