#ifndef __WORLD_H_INCLUDED__
#define __WORLD_H_INCLUDED__


#include <vector>

#include "precision.h"
#include "core.h"
#include "body.h"
#include "fgen.h"

class World
{
public:
	typedef std::vector<RigidBody*> RigidBodies;

protected:
	RigidBodies bodies;
	ForceRegistry registry;

public:
	// accesor
	RigidBodies& getRigidBodies();
	ForceRegistry& getForceRegistry();

	void startFrame();
	void integrate(real duration);
	void runPhysics(real duration);
};


#endif // __WORLD_H_INCLUDED__