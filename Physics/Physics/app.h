#ifndef __APP_H_INCLUDED__
#define __APP_H_INCLUDED__

#include <windows.h>
#include <GL/glut.h>
#include <iostream>

#include "precision.h"
#include "core.h"

#include "particle.h"
#include "pfgen.h"
#include "plinks.h"
#include "pcontacts.h"
#include "pworld.h"

#include "body.h"
#include "fgen.h"
#include "joints.h"
#include "world.h"
#include "collide_fine.h"

#include "graphics.h"


class ParticleApplication
{
public:
	static const int PARTICLE_NUM = 10;
	bool gravityOn;
	Vector2 g;

	ParticleWorld world;
	Particle particles[PARTICLE_NUM];

	ParticleGravity gravity;
	ParticleDrag drag;
	ParticleField field;

	ParticleControl control[PARTICLE_NUM];

	ParticleRod rod;

public:
	ParticleApplication();
	void update(real duration);
	void display();

	void setFieldSource(const Vector2& position);
	void switchGravity();
};


class RigidBodyApplication
{
protected:
	static const int MAX_CONTACT = 1000;
	static const int ITERATION = 1;

	World world;

	Gravity gravity;
	Aero aero;
	Field field;

	CollisionData collisionData;
	ContactResolver resolver;

protected:
	virtual void generateContacts() = 0;

public:
	RigidBodyApplication();
	void update(real duration);
	virtual void display() = 0;

	void passiveMotion(const Vector2& position);
	void keyboard(unsigned char key);

	static real sphereMOIPerMass(real radius);
	static real boixMOIPerMass(const Vector2& halfsize);
};


class SandBoxApp : public RigidBodyApplication
{
protected:
	static const int SPHERE_NUM = 5;
	static const int BOX_NUM = 5;
	static const int PLANE_NUM = 4;
	static const int JOINT_NUM = 2;

	RigidBody sphere_bodies[SPHERE_NUM];
	RigidBody box_bodies[BOX_NUM];
	
	CollisionSphere spheres[SPHERE_NUM];
	CollisionBox boxes[BOX_NUM];
	CollisionPlane planes[PLANE_NUM];

	Joint joints[JOINT_NUM];

protected:
	void generateContacts();

public:
	SandBoxApp();
	void display();
};


#endif // __APP_H_INCLUDED__