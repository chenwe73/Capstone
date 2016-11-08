#include "graphics.h"

const char* WINDOW_TITLE = "Physics";
const int WINDOW_X = 100;
const int WINDOW_Y = 100;
const int WINDOW_W = 600;
const int WINDOW_H = 600;
Vector2 mouseVector(0, 0);
real mouseRadius = 0.2;

void drawAxis()
{
	glBegin(GL_LINES);
		glVertex2f(0.0f, 1.0f);
		glVertex2f(0.0f, -1.0f);
		glVertex2f(1.0f, 0.0f);
		glVertex2f(-1.0f, 0.0f);
	glEnd();
}

void drawMouse()
{
	glPushMatrix();
	{
		glTranslatef(mouseVector.x, mouseVector.y, 0);
		glScalef(mouseRadius, mouseRadius, 1);
		drawCircle();
	}
	glPopMatrix();
}

// Draw a square
void drawSquare()
{
	// Draw the square
	glBegin(GL_POLYGON);
		glVertex2d(-1.0 / 2, -1.0 / 2);
		glVertex2d(+1.0 / 2, -1.0 / 2);
		glVertex2d(+1.0 / 2, +1.0 / 2);
		glVertex2d(-1.0 / 2, +1.0 / 2);
	glEnd();
}

// draw a N-vertex polygon with the array of vertex coordinate <x,y>
void drawPolygon(float x[], float y[], const int N)
{
	glBegin(GL_POLYGON);
	for (int i = 0; i < N; i++)
		glVertex2d(x[i], y[i]);
	glEnd();
}

// draw a circle
void drawCircle()
{
	const int N = 50; // number of triangle fans
	const float d = (2 * PI) / N;

	glBegin(GL_LINE_LOOP);
	//glVertex2d(0, 0);
	for (int i = 0; i < N + 1; i++)
		glVertex2d(cos(i * d), sin(i * d));
	glEnd();
}

void drawVector2(const Vector2& v)
{
	glBegin(GL_LINES);
	glVertex2f(0.0f, 0.0f);
	glVertex2f(v.x, v.y);
	glEnd();
}

void drawParticle(const Particle& p)
{
	glPushMatrix();
	{
		glTranslatef(p.getPosition().x, p.getPosition().y, 0);
		//drawVector2(p.getVelocity());
		glScalef(0.02, 0.02, 1);
		drawSquare();
	}
	glPopMatrix();
}

Vector2 screenToWorld(int x, int y)
{
	return Vector2 (((real)x / WINDOW_W * 2 - 1), (-(real)y / WINDOW_H * 2) + 1);
}

