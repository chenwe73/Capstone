#ifndef __GRAPHICS_H_INCLUDED__
#define __GRAPHICS_H_INCLUDED__


#include <windows.h>
#include <GL/glut.h>
#include "core.h"
#include "particle.h"

extern const char* WINDOW_TITLE;
extern const int WINDOW_X;
extern const int WINDOW_Y;
extern const int WINDOW_W;
extern const int WINDOW_H;

extern Vector2 mouseVector;
extern real mouseRadius;

void drawAxis();
void drawMouse();
void drawCircle();
void drawSquare();
void drawPolygon(float x[], float y[], const int N);
void drawVector2(const Vector2& v);
void drawParticle(const Particle& p);
Vector2 screenToWorld(int x, int y);


#endif // __GRAPHICS_H_INCLUDED__