#include <windows.h>  // for MS Windows
#include <GL/glut.h>  // GLUT, include glu.h and gl.h
#include <iostream>
#include <time.h>

#include "app.h"
#include "graphics.h"

using namespace std;

const char* WINDOW_TITLE = "Physics";
const int WINDOW_X = 700;
const int WINDOW_Y = 000;
const int WINDOW_W = 800;
const int WINDOW_H = 800;

clock_t t;
Vector2 mouseVector(100, 0);

void display();
void reshape(GLsizei width, GLsizei height);
void keyboard(unsigned char key, int x, int y);
void specialKeys(int key, int x, int y);
void mouse(int button, int state, int x, int y);
void passiveMotion(int x, int y);
void motion(int x, int y);
void idle();

void init();
void draw();
void pWorldInit();
void pWorldUpdate(real duration);
void worldInit();
void worldUpdate(real duration);
void drawParticleWorld();
void drawWorld();
Vector2 screenToWorld(int x, int y);

void test();

int main(int argc, char* argv[])
{
	//test();

	glutInit(&argc, argv);            // Initialize GLUT
	glutInitDisplayMode(GLUT_DOUBLE); // Enable double buffered mode
	glutInitWindowSize(WINDOW_W, WINDOW_H);  // Initial window width and height
	glutInitWindowPosition(WINDOW_X, WINDOW_Y); // Initial window top-left corner (x, y)
	glutCreateWindow(WINDOW_TITLE);      // Create window with given title
	init();                     // Our own OpenGL initialization

	glutDisplayFunc(display);     // Register callback handler for window re-paint
	glutReshapeFunc(reshape);     // Register callback handler for window re-shape
	glutIdleFunc(idle);
	glutKeyboardFunc(keyboard);   // Register callback handler for key event
	glutSpecialFunc(specialKeys); // Register callback handler for special-key event
	glutMouseFunc(mouse);   // Register callback handler for mouse event
	glutMotionFunc(motion);
	glutPassiveMotionFunc(passiveMotion);
	
	glutMainLoop();               // Enter event-processing loop
	return 0;
}

/* Initialize OpenGL Graphics */
void init() 
{
	glClearColor(0.0, 0.0, 0.0, 1.0); // Set background (clear) color
	pWorldInit();
	worldInit();
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
	case 'g':
		gravityOn = !gravityOn;
		break;

	case 'c':
		control1.on = !control1.on;
		control2.on = !control2.on;
		control3.on = !control3.on;
		control4.on = !control4.on;
		break;

	default:
		break;
	}
}

/* Callback handler for special-key event */
void specialKeys(int key, int x, int y) 
{
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

void idle()
{
	real duration = (real)(clock() - t) / CLOCKS_PER_SEC;
	t = clock();

	//duration = 1.0f / 60;
	//std::cin.ignore();
	//cout << 1.0f/duration << "  ";

	if (duration > 0)
	{
		duration = duration * 1;
		pWorldUpdate(duration);
		worldUpdate(duration);
	}

	glutPostRedisplay();    // Post a paint request to activate display()
}

void pWorldInit()
{
	pWorld.getParticles().push_back(&p1);
	pWorld.getParticles().push_back(&p2);
	pWorld.getParticles().push_back(&p3);
	pWorld.getParticles().push_back(&p4);

	ParticleWorld::Particles::iterator i = pWorld.getParticles().begin();
	for (; i != pWorld.getParticles().end(); i++)
	{
		pWorld.getForceRegistry().add((*i), &gravity);
		pWorld.getForceRegistry().add((*i), &drag);
		pWorld.getForceRegistry().add((*i), &field);
		pWorld.getForceRegistry().add((*i), &buoyancy);
	}

	//pWorld.getForceRegistry().add(&p1, &spring1);
	//pWorld.getForceRegistry().add(&p2, &spring2);
	//pWorld.getForceRegistry().add(&p1, &ancheredSpring);
	
	//pWorld.getForceRegistry().add(&p1, &bungee1);
	//pWorld.getForceRegistry().add(&p2, &bungee2);
	//pWorld.getForceRegistry().add(&p1, &ancheredBungee);
	//pWorld.getForceRegistry().add(&p1, &fakeSpring);
	//pWorld.getForceRegistry().add(&p3, &pointGravity);

	pWorld.getForceRegistry().add(&p1, &control1);
	pWorld.getForceRegistry().add(&p2, &control2);
	pWorld.getForceRegistry().add(&p3, &control3);
	pWorld.getForceRegistry().add(&p4, &control4);
	
	//pWorld.getContactGenerators().push_back(&cable);
	//pWorld.getContactGenerators().push_back(&rod1);
	//pWorld.getContactGenerators().push_back(&rod2);
	//pWorld.getContactGenerators().push_back(&rod3);
	//pWorld.getContactGenerators().push_back(&rod4);
}

void pWorldUpdate(real duration)
{
	field.setSource(mouseVector);
	if (gravityOn)
		gravity.setGravity(g);
	else
		gravity.setGravity(Vector2::ORIGIN);

	pWorld.startFrame();
	pWorld.runPhysics(duration);

	//p4.getPosition().print();
	//cout << "\n";
}

void worldInit()
{
	world.getRigidBodies().push_back(&body1);
	world.getRigidBodies().push_back(&body2);

	World::RigidBodies::iterator i = world.getRigidBodies().begin();
	for (; i != world.getRigidBodies().end(); i++)
	{
		world.getForceRegistry().add((*i), &gravityB);
		world.getForceRegistry().add((*i), &fieldB);
		//world.getForceRegistry().add((*i), &buoyancyB1);
		//world.getForceRegistry().add((*i), &buoyancyB2);
		world.getForceRegistry().add((*i), &aero);
	}

	//world.getForceRegistry().add(&body1, &springB1);
	//world.getForceRegistry().add(&body2, &springB2);

	collisionData.contactArray = new Contact[100];
	collisionData.restitution = 0.4;
	collisionData.friction = 0.5;
	
	sphere1.body = &body1;
	sphere1.radius = 0.2;
	sphere2.body = &body2;
	sphere2.radius = 0.2;
	plane.normal = Vector2::Y;
	plane.offset = waterHeight;
	box1.body = &body1;
	box1.halfSize = size;
	box2.body = &body2;
	box2.halfSize = size;
	RigidBody::setSleepEpsilon(0.00);
}

void worldUpdate(real duration)
{
	fieldB.setSource(mouseVector);
	if (gravityOn)
		gravityB.setGravity(g);
	else
		gravityB.setGravity(Vector2::ORIGIN);

	world.startFrame();
	world.runPhysics(duration);

	body1.getPosition().print();
	body1.getOrientation().print();

	collisionData.contacts = collisionData.contactArray;
	collisionData.contactsLeft = 100;
	collisionData.contactsCount = 0;
	int numContact = 0;
	numContact += CollisionDetector::sphereAndHalfSpace(sphere1, plane, &collisionData);
	numContact += CollisionDetector::boxAndHalfSpace(box2, plane, &collisionData);
	numContact += CollisionDetector::boxAndSphere(box2, sphere1, &collisionData);
	cout << " " << numContact << "\n";

	int jointContacts = 0;
	jointContacts += joint.addContact(collisionData.contacts, collisionData.contactsLeft);
	collisionData.addContacts(jointContacts);
	numContact += jointContacts;

	resolver.resolveContacts(collisionData.contactArray, numContact, duration);
}

void draw()
{
	glColor3f(1.0f, 1.0f, 1.0f);
	drawAxis();

	drawParticleWorld();
	drawWorld();
	
	glColor3f(0.0f, 0.0f, 1.0f);
	drawMouse(mouseVector, mouseRadius);
	drawLine(Vector2(-1, waterHeight), Vector2(1, waterHeight));
	//drawTrace(&p1);
}

void drawParticleWorld()
{
	glColor3f(0.0f, 1.0f, 0.0f);
	ParticleWorld::Particles::iterator i = pWorld.getParticles().begin();
	for (; i != pWorld.getParticles().end(); i++)
		drawParticle(*i);

	glColor3f(0.0f, 0.0f, 1.0f);
	drawParticleLink(&rod1);
	drawParticleLink(&rod2);
	drawParticleLink(&rod3);
	drawParticleLink(&rod4);
	//drawTrace(&p3);
}

void drawWorld()
{
	glColor3f(1.0f, 0.0f, 0.0f);
	drawRigidBody(&body1);
	drawCollisionSphere(&sphere1);
	//drawCollisionBox(&box1);

	drawRigidBody(&body2);
	//drawCollisionSphere(&sphere2);
	drawCollisionBox(&box2);

	glColor3f(0.0f, 0.0f, 1.0f);
	//drawSpring(&body1, &springB1);
	drawCollisionData(&collisionData);
}

Vector2 screenToWorld(int x, int y)
{
	return Vector2(((real)x / WINDOW_W * 2 - 1), (-(real)y / WINDOW_H * 2) + 1);
}

void test()
{
	
}