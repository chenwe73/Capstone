#ifndef __APP_H_INCLUDED__
#define __APP_H_INCLUDED__


#include "precision.h"
#include "core.h"

#include "particle.h"
#include "pfgen.h"
#include "plinks.h"
#include "pcontacts.h"
#include "pworld.h"

#include "body.h"
#include "fgen.h"
#include "world.h"

#include "collide_fine.h"

Vector2 origin(0, 0);
real mouseRadius = 0.2;
real waterHeight = -0.8f;
Vector2 g(0, -0.2);
bool gravityOn = false;
bool controlOn = false;

ParticleWorld pWorld(100, 0);

Particle p1(Vector2(0.0, -0.2), 1);
Particle p2(Vector2(0.0, 0.0), 1);
Particle p3(Vector2(0.2, 0.0), 1);
Particle p4(Vector2(0.4, 0.2), 1);

ParticleGravity gravity(g);
ParticleDrag drag(0.2, 0.2);
ParticleField field(Vector2::ORIGIN, mouseRadius, 0.01);
ParticleBuoyancy buoyancy(0.0, 0.0005, waterHeight, 1000);
ParticlePointGravity pointGravity(Vector2::ORIGIN, 0.01);

ParticleSpring spring1(&p2, 5, 0.2);
ParticleSpring spring2(&p1, 5, 0.2);
ParticleAnchoredSpring ancheredSpring(&origin, 5, 0.2);
ParticleBungee bungee1(&p2, 5, 0.2);
ParticleBungee bungee2(&p1, 5, 0.2);
ParticleAnchoredBungee ancheredBungee(&origin, 5, 0.2);
ParticleFakeSpring fakeSpring(&origin, 100, 10);

Vector2 ox(0.2, 0.0);
Vector2 oy(0.0, 0.2);
Vector2 offset[2] = { origin, origin };

Particle* other1[2] = { &p2, &p4 };
Vector2 offset1[2] = { ox, oy };
ParticleControl control1(other1, offset1);
Particle* other2[2] = { &p1, &p3 };
Vector2 offset2[2] = { -ox, oy };
ParticleControl control2(other2, offset2);
Particle* other3[2] = { &p4, &p2 };
Vector2 offset3[2] = { -ox, -oy };
ParticleControl control3(other3, offset3);
Particle* other4[2] = { &p3, &p1 };
Vector2 offset4[2] = { ox, -oy };
ParticleControl control4(other4, offset4);

ParticleCable cable(&p1, &p2, 0.2, 0.1);
ParticleRod rod1(&p1, &p2, 0.2);
ParticleRod rod2(&p2, &p3, 0.2);
ParticleRod rod3(&p3, &p4, 0.2);
ParticleRod rod4(&p4, &p1, 0.2);


World world;

RigidBody body1(Vector2(0, 0), Vector2(1, 0.5),
	1, 1.0f / (1.0f / 1 * 0.2 * 0.2 / 2));
RigidBody body2(Vector2(0, 0.3), Vector2(-1, 0),
	1, 1.0f / (1.0f / 1 * 0.2 * 0.2 / 2));

Vector2 connectionPoint(0.2, 0.0);
Vector2 size(0.2, 0.1);

Gravity gravityB(g);
Field fieldB(Vector2::ORIGIN, mouseRadius, 0.01, connectionPoint);
Spring springB1(connectionPoint, &body2, connectionPoint, 5, 0.2);
Spring springB2(connectionPoint, &body1, connectionPoint, 5, 0.2);

Matrix2 aeroTensor(-0.1, 0, 0, -0.1);
Vector2 windSpeed = Vector2::ORIGIN;
Aero aero(aeroTensor, Vector2::ORIGIN, &windSpeed);
Buoyancy buoyancyB1(0.0, 0.005, waterHeight, 1000, Vector2::ORIGIN);
Buoyancy buoyancyB2(0.0, 0.0005, waterHeight, 1000, -connectionPoint);

CollisionData collisionData;
CollisionSphere sphere1;
CollisionSphere sphere2;
CollisionPlane plane;
CollisionBox box1;
CollisionBox box2;


#endif // __APP_H_INCLUDED__