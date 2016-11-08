#include <windows.h>  // for MS Windows
#include <GL/glut.h>  // GLUT, include glu.h and gl.h
#include <iostream>
#include <time.h>

#include "precision.h"
#include "core.h"
#include "particle.h"
#include "graphics.h"
#include "pfgen.h"

using namespace std;

void initGL();
void display();
void reshape(GLsizei width, GLsizei height);
void keyboard(unsigned char key, int x, int y);
void specialKeys(int key, int x, int y);
void mouse(int button, int state, int x, int y);
void passiveMotion(int x, int y);
void motion(int x, int y);

void init();
void update();
void draw();

clock_t t;
Vector2 v1(0.1, 0.4);
Vector2 v2(0.0, 0.0);
Particle p1(Vector2(0, 0.5), Vector2(-0.1, 0.1), Vector2(0, 0), 0.99, 1, Vector2(0, 0));
ParticleForceRegistry registry;
ParticleGravity gravity(Vector2(0, -0.2));
ParticleDrag drag(0.2, 0.2);
ParticleField field(Vector2(0, 0), mouseRadius, 0.01);
ParticleAnchoredSpring ancheredSpring(&v2, 5, 0.2);


int main(int argc, char* argv[])
{
	void test();
	//test();
	init();

	glutInit(&argc, argv);            // Initialize GLUT
	glutInitDisplayMode(GLUT_DOUBLE); // Enable double buffered mode
	glutInitWindowSize(WINDOW_W, WINDOW_H);  // Initial window width and height
	glutInitWindowPosition(WINDOW_X, WINDOW_Y); // Initial window top-left corner (x, y)
	glutCreateWindow(WINDOW_TITLE);      // Create window with given title
	initGL();                     // Our own OpenGL initialization

	glutDisplayFunc(display);     // Register callback handler for window re-paint
	glutReshapeFunc(reshape);     // Register callback handler for window re-shape
	glutIdleFunc(update);
	glutKeyboardFunc(keyboard);   // Register callback handler for key event
	glutSpecialFunc(specialKeys); // Register callback handler for special-key event
	glutMouseFunc(mouse);   // Register callback handler for mouse event
	glutMotionFunc(motion);
	glutPassiveMotionFunc(passiveMotion);
	
	glutMainLoop();               // Enter event-processing loop
	return 0;
}

/* Initialize OpenGL Graphics */
void initGL() 
{
	glClearColor(0.0, 0.0, 0.0, 1.0); // Set background (clear) color
}

void display() 
{
	glClear(GL_COLOR_BUFFER_BIT);  // Clear the color buffer
	glMatrixMode(GL_MODELVIEW);    // To operate on the model-view matrix
	glLoadIdentity();              // Reset model-view matrix

	draw();

	glFlush();
	glutSwapBuffers();  // Swap front and back buffers (of double buffered mode)
}

/* Call back when the windows is re-sized */
void reshape(GLsizei width, GLsizei height) {}

/* Callback handler for normal-key event */
void keyboard(unsigned char key, int x, int y)
{
	switch (key)
	{
		case 27:     // ESC key
			exit(0);
			break;
		default:
			break;
	}
}

/* Callback handler for special-key event */
void specialKeys(int key, int x, int y) 
{
	switch (key) 
	{
		case GLUT_KEY_F1:
			exit(0);
			break;
		default:
			break;
	}
}

/* Callback handler for mouse event */
void mouse(int button, int state, int x, int y)
{
	if (button == GLUT_RIGHT_BUTTON && state == GLUT_UP)
	{
		exit(0);
	}
}

// where (x, y) is the mouse location in Window's coordinates
void motion(int x, int y)
{
	mouseVector = screenToWorld(x, y);
}

void passiveMotion(int x, int y)
{
	mouseVector = screenToWorld(x, y);
}

void init()
{
	registry.add(&p1, &gravity);
	registry.add(&p1, &drag);
	registry.add(&p1, &field);
	registry.add(&p1, &ancheredSpring);
}

void update()
{
	real time = (real)(clock() - t) / CLOCKS_PER_SEC;
	t = clock();

	v1.rotate(time);
	field.setSource(mouseVector);
	registry.updateForces(time);
	p1.integrate(time);

	glutPostRedisplay();    // Post a paint request to activate display()

	p1.getPosition().print();
	cout << "\n";
}

void draw()
{
	glColor3f(0.0f, 0.0f, 1.0f);
	drawMouse();

	glColor3f(1.0f, 1.0f, 1.0f);
	drawAxis();

	glColor3f(1.0f, 0.0f, 0.0f);
	//drawVector2(v1);

	glColor3f(0.0f, 1.0f, 0.0f);
	drawParticle(p1);
	drawVector2(p1.getPosition());
}

void test()
{
	real a = 2;
	Vector2 v1(1, 2);
	v1.print();
	Vector2 v2;
	v2.print();
	cout << "\n";

	v2 = v1*a;

	v1.print();
	v2.print();
	cout << v1*v2;
}

