#ifndef __GRAPHICS_H_INCLUDED__
#define __GRAPHICS_H_INCLUDED__


#include <windows.h>
#include <GL/glut.h>

#include "precision.h"
#include "core.h"
#include "particle.h"
#include "body.h"
#include "fgen.h"

#include "collide_fine.h"
#include "contacts.h"


void drawAxis();
void drawMouse(Vector2 position, real radius);
void drawCircle();
void drawSquare();
void drawPolygon(float x[], float y[], const int N);
void drawVector2(const Vector2& v);
void drawLine(const Vector2& v1, const Vector2& v2);
void drawParticle(Particle *p);
void drawRigidBody(RigidBody *body);
void drawSpring(RigidBody *body, Spring *spring);
void drawContact(Contact *contact);
void drawCollisionData(CollisionData *data);
void drawCollisionSphere(CollisionSphere *sphere);
void drawCollisionBox(CollisionBox *box);
void drawTrace(Particle *p);


#endif // __GRAPHICS_H_INCLUDED__