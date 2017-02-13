#ifndef __JOINTS_H_INCLUDED__
#define __JOINTS_H_INCLUDED__


#include "precision.h"
#include "core.h"
#include "body.h"

#include "contacts.h"

class Joint : public ContactGenerator
{
public:
	RigidBody* body[2];
	Vector2 position[2];
	real error;

public:
	Joint(RigidBody *a, const Vector2& a_pos,
		RigidBody *b, const Vector2& b_pos, real error);
	int addContact(Contact *contact, int limit) const;
};


#endif // __JOINTS_H_INCLUDED__