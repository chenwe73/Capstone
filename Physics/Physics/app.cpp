#include "app.h"

ParticleApplication::ParticleApplication() :
gravityOn(false),
g(0, -0.2),
world(100, 0),
gravity(Vector2::ORIGIN),
drag(0.2, 0.2),
field(Vector2(100, 0), 0.2, 0.01)
{
	particles[0] = Particle(Vector2(0.5, 0.2), 1);
	particles[1] = Particle(Vector2(0.5, 0.0), 1);

	for (int i = 2; i < PARTICLE_NUM; i++)
		particles[i] = Particle(Vector2((real)i / 10 - 1.0, -0.2), 1);

	rod = ParticleRod(&(particles[0]), &(particles[1]), 0.2);

	for (int i = 0; i < PARTICLE_NUM-1; i++)
		control[i] = ParticleControl(&particles[i+1]);
	control[PARTICLE_NUM - 1] = ParticleControl(&particles[0]);

	// assign to world
	for (int i = 0; i < PARTICLE_NUM; i++)
		world.getParticles().push_back(&particles[i]);

	ParticleWorld::Particles::iterator i = world.getParticles().begin();
	for (; i != world.getParticles().end(); i++)
	{
		world.getForceRegistry().add((*i), &gravity);
		world.getForceRegistry().add((*i), &drag);
		world.getForceRegistry().add((*i), &field);
	}

	for (int i = 0; i < PARTICLE_NUM; i++)
		world.getForceRegistry().add(&particles[i], &control[i]);

	//world.getContactGenerators().push_back(&rod);
}

void ParticleApplication::update(real duration)
{
	world.startFrame();
	world.runPhysics(duration);
}

void ParticleApplication::display()
{
	glColor3fv(OBJECT_COLOR);
	ParticleWorld::Particles::iterator i = world.getParticles().begin();
	for (; i != world.getParticles().end(); i++)
		drawParticle(*i);

	glColor3fv(SECONDARY_COLOR);
	drawParticleLink(&rod);
	drawParticleField(&field);
}

void ParticleApplication::switchGravity()
{
	gravityOn = !gravityOn;
	if (gravityOn)
		gravity.setGravity(g);
	else
		gravity.setGravity(Vector2::ORIGIN);
}

void ParticleApplication::setFieldSource(const Vector2& position)
{
	field.setSource(position);
}


RigidBodyApplication::RigidBodyApplication() :
world(MAX_CONTACT, 0),
gravity(Vector2(0, -0.4), true),
aero(Matrix2(-0.1, 0, 0, -0.1), Vector2::ORIGIN, &Vector2::ORIGIN),
field(Vector2(100, 0), 0.2, 0.05, Vector2(0.0, 0.0)),
collisionData(MAX_CONTACT, 0.4, 0.5),
resolver(MAX_CONTACT, MAX_CONTACT, -0.0, -0.0)
{
	RigidBody::setSleepEpsilon(0.001);
	collisionData.reset();
}

void RigidBodyApplication::update(real duration)
{
	for (int i = 0; i < ITERATION; i++)
	{
		world.startFrame();
		world.runPhysics(duration / ITERATION);

		generateContacts();
		resolver.setIterations(collisionData.contactsCount * 2, collisionData.contactsCount * 2);
		resolver.resolveContacts(collisionData.contactArray,
			collisionData.contactsCount, duration);
		//std::cout << collisionData.contactsCount << "\n";
	}
}

void RigidBodyApplication::keyboard(unsigned char key)
{
	switch (key)
	{
	case 'g':
		gravity.setOn(!gravity.on);
		break;

	default:
		break;
	}
}

void RigidBodyApplication::passiveMotion(const Vector2& position)
{
	field.setSource(position);
}

real RigidBodyApplication::sphereMOIPerMass(real radius)
{
	return 2.0 / 5 * radius * radius;
}

real RigidBodyApplication::boixMOIPerMass(const Vector2& halfsize)
{
	return 1.0 / 3 *(halfsize.x * halfsize.x + halfsize.y * halfsize.y);
}



SandBoxApp::SandBoxApp() :
RigidBodyApplication()
{
	real sphereMass = 1;
	real sphereRadius = 0.1;
	real sphereMOI = sphereMass * sphereMOIPerMass(sphereRadius);
	for (int i = 0; i < SPHERE_NUM; i++)
	{
		sphere_bodies[i] = RigidBody(Vector2(-0.5, -0.5 + (real)i * sphereRadius * 2),
			Vector2::X, 1.0f / sphereMass, 1.0f / sphereMOI);
	}

	real boxMass = 1;
	Vector2 boxHalfSize(0.1, 0.1);
	real boxMOI = boxMass * boixMOIPerMass(boxHalfSize);
	for (int i = 0; i < BOX_NUM; i++)
	{
		box_bodies[i] = RigidBody(Vector2(0.5, -0.5 + (real)i * boxHalfSize.y * 2),
			Vector2(1, 0.0), 1.0f / boxMass, 1.0f / boxMOI);
	}

	// initialize collision contacts
	for (int i = 0; i < SPHERE_NUM; i++)
		spheres[i] = CollisionSphere(&sphere_bodies[i], sphereRadius);

	for (int i = 0; i < BOX_NUM; i++)
		boxes[i] = CollisionBox(&box_bodies[i], boxHalfSize);

	real wallDist = 0.9;
	planes[0] = CollisionPlane(Vector2::Y, -wallDist);
	planes[1] = CollisionPlane(-Vector2::Y, -wallDist);
	planes[2] = CollisionPlane(Vector2::X, -wallDist * 2);
	planes[3] = CollisionPlane(-Vector2::X, -wallDist * 2);

	joints[0] = Joint(&sphere_bodies[0], Vector2(0, 0.0), 
		&box_bodies[0], Vector2(-0.3, -0.1), 0);
	joints[1] = Joint(&sphere_bodies[1], Vector2(0, 0.0), 
		&box_bodies[0], Vector2(+0.3, -0.1), 0);

	// assign to world
	for (int i = 0; i < SPHERE_NUM; i++)
		world.getRigidBodies().push_back(&sphere_bodies[i]);
	for (int i = 0; i < BOX_NUM; i++)
		world.getRigidBodies().push_back(&box_bodies[i]);

	World::RigidBodies::iterator i = world.getRigidBodies().begin();
	for (; i != world.getRigidBodies().end(); i++)
	{
		world.getForceRegistry().add((*i), &gravity);
		world.getForceRegistry().add((*i), &field);
		world.getForceRegistry().add((*i), &aero);
	}
	
	//for (int i = 0; i < JOINT_NUM; i++)
	//	world.getContactGenerators().push_back(&joints[i]);
}

void SandBoxApp::generateContacts()
{
	collisionData.reset();

	for (int i = 0; i < SPHERE_NUM; i++)
		for (int j = i + 1; j < SPHERE_NUM; j++)
			CollisionDetector::sphereAndSphere(spheres[i], spheres[j], &collisionData);

	for (int i = 0; i < BOX_NUM; i++)
		for (int j = i + 1; j < BOX_NUM; j++)
			CollisionDetector::boxAndBox2(boxes[i], boxes[j], &collisionData);

	for (int i = 0; i < BOX_NUM; i++)
		for (int j = 0; j < SPHERE_NUM; j++)
			CollisionDetector::boxAndSphere(boxes[i], spheres[j], &collisionData);

	for (int i = 0; i < PLANE_NUM; i++)
	{
		for (int j = 0; j < SPHERE_NUM; j++)
			CollisionDetector::sphereAndHalfSpace(spheres[j], planes[i], &collisionData);
		for (int j = 0; j < BOX_NUM; j++)
			CollisionDetector::boxAndHalfSpace(boxes[j], planes[i], &collisionData);
	}
}

void SandBoxApp::display()
{
	World::RigidBodies::iterator i = world.getRigidBodies().begin();
	for (; i != world.getRigidBodies().end(); i++)
		drawRigidBody(*i);

	glColor3fv(OBJECT_COLOR);
	for (int i = 0; i < SPHERE_NUM; i++)
		drawCollisionSphere(&spheres[i]);
	for (int i = 0; i < BOX_NUM; i++)
		drawCollisionBox(&boxes[i]);
	for (int i = 0; i < PLANE_NUM; i++)
		drawCollisionPlane(&planes[i]);

	glColor3fv(SECONDARY_COLOR);
	drawField(&field);

	drawCollisionData(&collisionData);
}