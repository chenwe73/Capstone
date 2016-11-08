#include "pfgen.h"

void ParticleForceRegistry::add(Particle *particle, ParticleForceGenerator *fg)
{
	ParticleForceRegistration registration;
	registration.particle = particle;
	registration.fg = fg;
	registrations.push_back(registration);
}

void ParticleForceRegistry::updateForces(real duration)
{
	Registry::iterator i = registrations.begin();
	for (; i != registrations.end(); i++)
	{
		i->fg->updateForce(i->particle, duration);
	}
}

ParticleGravity::ParticleGravity(const Vector2& g)
{
	gravity = g;
}

void ParticleGravity::updateForce(Particle *particle, real duration)
{
	if (!particle->hasFiniteMass())
		return;

	// Fg = m*g
	particle->addForce(gravity * particle->getMass());
}

ParticleDrag::ParticleDrag(real k1, real k2)
{
	this->k1 = k1;
	this->k2 = k2;
}

void ParticleDrag::updateForce(Particle *particle, real duration)
{
	Vector2 force;
	Vector2 v = particle->getvelocity();

	// Fdrag = -norm(v)(k1*abs(v) + k2*abs(v)^2)
	real speed = v.magnitude();
	real dragCoeff = k1 * speed + k2 * speed * speed;
	force = v.unit() * -dragCoeff;

	particle->addForce(force);
}

ParticleField::ParticleField(const Vector2& s, real r, real k_p)
{
	source = s;
	radius = r;
	k = k_p;
}

void ParticleField::updateForce(Particle *particle, real duration)
{
	Vector2 force;
	Vector2 v = particle->getPosition() - source;
	real d = v.magnitude();
	if (d == 0)
		return;
	if (d <= radius)
	{
		force = v.unit() * (k / (d*d));
		particle->addForce(force);
	}
}

void ParticleField::setSource(const Vector2& s)
{
	source = s;
}

ParticleSpring::ParticleSpring(Particle *o, real sc, real rl)
{
	other = o;
	springConstant = sc;
	restLength = rl;
}

void ParticleSpring::updateForce(Particle *particle, real duration)
{
	Vector2 force;
	Vector2 v = particle->getPosition() - other->getPosition();
	real d = v.magnitude();

	// F = -k * abs(l - l0)
	force = v.unit() * -real_abs(d - restLength) * springConstant;
	particle->addForce(force);
}

ParticleAnchoredSpring::ParticleAnchoredSpring(Vector2 *a, real sc, real rl)
{
	anchor = a;
	springConstant = sc;
	restLength = rl;
}

void ParticleAnchoredSpring::updateForce(Particle *particle, real duration)
{
	Vector2 force;
	Vector2 v = particle->getPosition() - *anchor;
	real d = v.magnitude();

	// F = -k * abs(l - l0)
	force = v.unit() * -real_abs(d - restLength) * springConstant;
	particle->addForce(force);
}