#include <windows.h>  // for MS Windows
#include <GL/glut.h>  // GLUT, include glu.h and gl.h
#include <iostream>
#include <time.h>

#include "graphics.h"
#include "app.h"
#include "car.h"
#include "domino.h"
#include "cradle.h"
#include "pool.h"

using namespace std;

const char* WINDOW_TITLE = "Physics";
const int WINDOW_X = 0;
const int WINDOW_Y = 0;
const int WINDOW_W = 1600;
const int WINDOW_H = 800;

const double refreshMills = 1;

clock_t t;
PoolApp app;

void display();
void reshape(GLsizei width, GLsizei height);
void keyboard(unsigned char key, int x, int y);
void specialKeys(int key, int x, int y);
void mouse(int button, int state, int x, int y);
void passiveMotion(int x, int y);
void motion(int x, int y);
void Timer(int value);

void init();
void draw();
Vector2 screenToWorld(int x, int y);


int main(int argc, char* argv[])
{
	glutInit(&argc, argv);            // Initialize GLUT
	glutInitDisplayMode(GLUT_DEPTH | GLUT_MULTISAMPLE); // Enable double buffered mode
	glutInitWindowSize(WINDOW_W, WINDOW_H);  // Initial window width and height
	glutInitWindowPosition(WINDOW_X, WINDOW_Y); // Initial window top-left corner (x, y)
	glutCreateWindow(WINDOW_TITLE);      // Create window with given title
	init();

	glutDisplayFunc(display);     // Register callback handler for window re-paint
	glutReshapeFunc(reshape);     // Register callback handler for window re-shape
	glutKeyboardFunc(keyboard);   // Register callback handler for key event
	glutSpecialFunc(specialKeys); // Register callback handler for special-key event
	glutMouseFunc(mouse);   // Register callback handler for mouse event
	glutMotionFunc(motion);
	glutPassiveMotionFunc(passiveMotion);
	glutTimerFunc(0, Timer, 0);     // First timer call immediately
	
	glutMainLoop();               // Enter event-processing loop
	return 0;
}


void init() 
{
	glClearColor(0.0, 0.0, 0.0, 1.0); // Set background (clear) color
	glMatrixMode(GL_MODELVIEW);    // To operate on the model-view matrix
	glLoadIdentity();              // Reset model-view matrix
	GLdouble aspect = (GLdouble)WINDOW_W / WINDOW_H;
	gluOrtho2D(-1.0 * aspect, 1.0 * aspect, -1.0, 1.0);
	glEnable(GL_LINE_SMOOTH);
	//glEnable(GL_MULTISAMPLE);
}

void display() 
{
	glClear(GL_COLOR_BUFFER_BIT);  // Clear the color buffer
	draw();

	glFlush();
	glutSwapBuffers();  // Swap front and back buffers (of double buffered mode)
}

/* Call back when the windows is re-sized */
void reshape(GLsizei width, GLsizei height) 
{}

/* Callback handler for normal-key event */
void keyboard(unsigned char key, int x, int y)
{
	app.keyboard(key);
}

/* Callback handler for special-key event */
void specialKeys(int key, int x, int y) 
{}

/* Callback handler for mouse event */
void mouse(int button, int state, int x, int y)
{
	if (button == GLUT_RIGHT_BUTTON && state == GLUT_UP)
		exit(0);
}

// where (x, y) is the mouse location in Window's coordinates
void motion(int x, int y)
{}

void passiveMotion(int x, int y)
{
	Vector2 mouseVector = screenToWorld(x, y);
	app.passiveMotion(mouseVector);
}

/* Called back when timer expired */
void Timer(int value)
{
	real duration = (real)(clock() - t) / CLOCKS_PER_SEC;
	t = clock();
	//std::cout << duration << "\n";

	if (0)
	{
		duration = 1.0f / 60;
		cin.ignore();
	}

	if (duration > 0)
	{
		duration = duration * 1;
		app.update(duration);
	}

	glutPostRedisplay();      // Post re-paint request to activate display()
	glutTimerFunc(refreshMills, Timer, 0); // next Timer call milliseconds later
}

void draw()
{
	glColor3fv(SECONDARY_COLOR);
	//drawAxis();

	app.display();
}

Vector2 screenToWorld(int x, int y)
{
	return Vector2(((real)x / WINDOW_W * 4 - 2), (-(real)y / WINDOW_H * 2) + 1);
}
