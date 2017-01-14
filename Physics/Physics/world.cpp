#include "world.h"

World::RigidBodies& World::getRigidBodies()
{
	return bodies;
}

/*World::ContactGenerators& World::getContactGenerators()
{
	return contactGenerators;
}*/

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