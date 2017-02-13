#include "joints.h"

Joint::Joint(RigidBody *a, const Vector2& a_pos,
	RigidBody *b, const Vector2& b_pos, real error)
{
	body[0] = a;
	body[1] = b;
	position[0] = a_pos;
	position[1] = b_pos;
	Joint::error = error;
}

int Joint::addContact(Contact *contact, int limit) const
{
	Vector2 a_pos_world = body[0]->getPointInWorldSpace(position[0]);
	Vector2 b_pos_world = body[1]->getPointInWorldSpace(position[1]);

	Vector2 normal = b_pos_world - a_pos_world;
	real penetration = normal.magnitude() - error;

	if (penetration <= 0)
		return 0;

	contact->setBodyData(body[0], body[1], 1, 0);
	contact->contactPoint = (a_pos_world + b_pos_world) * 0.5f;
	contact->contactNormal = normal.unit();
	contact->penetration = penetration;

	return 1;
}