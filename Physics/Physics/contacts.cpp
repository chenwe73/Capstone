#include "contacts.h"

void Contact::setBodyData(RigidBody* body1, RigidBody* body2,
	real friction, real restitution)
{
	this->body[0] = body1;
	this->body[1] = body2;
	this->friction = friction;
	this->restitution = restitution;
}

void Contact::calculateInternals(real duration)
{
	contactNormal.normalize();
	if (body[0] == NULL)
		swapBodies();

	calculateContactBasis();

	relativeContactPosition[0] = contactPoint - body[0]->getPosition();
	if (body[1] != NULL)
		relativeContactPosition[1] = contactPoint - body[1]->getPosition();

	contactVelocity = calculateLocalVelocity(0, duration);
	if (body[1] != NULL)
		contactVelocity.minus(calculateLocalVelocity(1, duration));

	calculateDesiredDeltaVelocity(duration);
}

void Contact::swapBodies()
{
	contactNormal.invert();
	RigidBody* temp = body[0];
	body[0] = body[1];
	body[1] = temp;
}

void Contact::calculateContactBasis()
{
	Vector2 tangent = contactNormal.crossProduct(-1);
	contactToWorld.setComponents(contactNormal, tangent);
}

void Contact::calculateDesiredDeltaVelocity(real duration)
{
	desiredDeltaVelocity = -contactVelocity.x * (1 + restitution);
}

Vector2 Contact::calculateLocalVelocity(int bodyIndex, real duration)
{
	RigidBody *thisBody = body[bodyIndex];
	Vector2 velocityRot = relativeContactPosition[bodyIndex].crossProduct
		(-thisBody->getAngularVelocity());
	Vector2 velocityNet = velocityRot + thisBody->getVelocity();

	return contactToWorld.transpose() * velocityNet;
}

Vector2 Contact::calculateFrictionLessImpulse()
{
	real deltaVelLocal = 0;
	for (int i = 0; i < 2; i++)
	{
		if (body[i] != NULL)
		{
			Vector2 unitImpulse = contactNormal;
			real angularImpulse = relativeContactPosition[i].crossProduct(unitImpulse);
			real deltaAngularVelocity = angularImpulse * body[i]->getInverseMomentOfInertia();
			Vector2 deltaVelRotWorld = relativeContactPosition[i].crossProduct(-deltaAngularVelocity);
			deltaVelLocal += deltaVelRotWorld * contactNormal + body[i]->getInverseMass();
		}
	}

	Vector2 impulseContact;
	impulseContact.x = desiredDeltaVelocity / deltaVelLocal;
	impulseContact.y = 0;
	return impulseContact;
}

void Contact::applyImpulse()
{
	if (contactVelocity.x >= 0)
		return;

	Vector2 impulse = contactToWorld * calculateFrictionLessImpulse();
	body[0]->applyImpulseAtPoint(impulse, contactPoint);
	if (body[1] != NULL)
	{
		impulse.invert();
		body[1]->applyImpulseAtPoint(impulse, contactPoint);
	}
}

void Contact::applypositionchange()
{
	real linearMove[2];
	real angularMove[2];
	real totalInertia = 0;
	real linearInertia[2];
	real angularInertia[2];
	real deltaAngularVelocity[2];

	for (int i = 0; i < 2; i++)
		if (body[i] != NULL)
		{
			real angularImpulse = relativeContactPosition[i].crossProduct(contactNormal);
			deltaAngularVelocity[i] = angularImpulse * body[i]->getInverseMomentOfInertia();
			Vector2 deltaVelRotWorld = relativeContactPosition[i].crossProduct(-deltaAngularVelocity[i]);

			angularInertia[i] = deltaVelRotWorld * contactNormal;
			linearInertia[i] = body[i]->getInverseMass();
			totalInertia += angularInertia[i] + linearInertia[i];
		}

	real angularLimitConstant = 0.2f;
	// apply displacement and rotation
	for (int i = 0; i < 2; i++)
		if (body[i] != NULL)
		{
			int sign = 1 - i * 2;
			linearMove[i] = sign * penetration * (linearInertia[i] / totalInertia);
			angularMove[i] = sign * penetration * (angularInertia[i] / totalInertia);

			real limit = angularLimitConstant * relativeContactPosition[i].magnitude();
			if (real_abs(angularMove[i]) > limit)
			{
				real totalMove = linearMove[i] + angularMove[i];
				if (angularMove[i] >= 0)
					angularMove[i] = limit;
				else
					angularMove[i] = -limit;
				linearMove[i] = totalMove - angularMove[i];
			}

			linearChange[i] = contactNormal * linearMove[i];
			angularChange[i] = deltaAngularVelocity[i] / angularInertia[i] * angularMove[i];
			body[i]->move(linearChange[i]);
			body[i]->rotate(angularChange[i]);
			body[i]->calculateDerivedData();
		}
}

void ContactResolver::resolveContacts(Contact *contactArray,
	int numContacts, real duration)
{
	if (numContacts == 0)
		return;

	prepareContacts(contactArray, numContacts, duration);
	adjestPositions(contactArray, numContacts, duration);
	adjustVelocities(contactArray, numContacts, duration);
}

void ContactResolver::prepareContacts(Contact *contactArray,
	int numContacts, real duration)
{
	for (int i = 0; i < numContacts; i++)
		contactArray[i].calculateInternals(duration);
}

void ContactResolver::adjestPositions(Contact *contactArray,
	int numContacts, real duration)
{
	Vector2 linearChange[2], angularChange[2];

	positionIterationUsed = 0;
	while (positionIterationUsed < positionIteration)
	{
		real max = positionEpsilon;
		int indexMax = -1;

		for (int i = 0; i < numContacts; i++)
		{
			if (contactArray[i].penetration > max)
			{
				max = contactArray[i].penetration;
				indexMax = i;
			}
		}
		if (indexMax == -1)
			break;
		contactArray[indexMax].applypositionchange();

		// todo update penetrations

		positionIterationUsed++;
	}
}

void ContactResolver::adjustVelocities(Contact *contactArray,
	int numContacts, real duration)
{

}