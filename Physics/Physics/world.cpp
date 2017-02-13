#include "world.h"

World::World(int maxContcts, int iterations)
	: resolver(iterations, iterations)
{
	World::maxContacts = maxContcts;
	contacts = new Contact[maxContcts];
	calculateIterations = (iterations == 0);
}

World::~World()
{
	delete[] contacts;
}

World::RigidBodies& World::getRigidBodies()
{
	return bodies;
}

World::ContactGenerators& World::getContactGenerators()
{
	return contactGenerators;
}

ForceRegistry& World::getForceRegistry()
{
	return registry;
}

void World::startFrame()
{
	RigidBodies::iterator i = bodies.begin();
	for (; i != bodies.end(); i++)
	{
		(*i)->clearAccumulators();
		(*i)->calculateDerivedData();
	}
}

void World::integrate(real duration)
{
	RigidBodies::iterator i = bodies.begin();
	for (; i != bodies.end(); i++)
		(*i)->integrate(duration);
}

void World::runPhysics(real duration)
{
	registry.updateForces(duration);
	integrate(duration);
	/*
	int usedContacts = generateContacts();
	//std::cout << usedContacts;
	if (calculateIterations)
		resolver.setIterations(usedContacts * 2);
	resolver.resolveContacts(contacts, usedContacts, duration);
	*/
}