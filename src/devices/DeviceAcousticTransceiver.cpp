/*----------------------------------------------------------------------------*/
/*    Copyright (C) 2011-2012 Alexandre Campo                                 */
/*                                                                            */
/*    This file is part of FaMouS  (a fast, modular and simple simulator).    */
/*                                                                            */
/*    FaMouS is free software: you can redistribute it and/or modify          */
/*    it under the terms of the GNU General Public License as published by    */
/*    the Free Software Foundation, either version 3 of the License, or       */
/*    (at your option) any later version.                                     */
/*                                                                            */
/*    FaMouS is distributed in the hope that it will be useful,               */
/*    but WITHOUT ANY WARRANTY; without even the implied warranty of          */
/*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           */
/*    GNU General Public License for more details.                            */
/*                                                                            */
/*    You should have received a copy of the GNU General Public License       */
/*    along with FaMouS.  If not, see <http://www.gnu.org/licenses/>.         */
/*----------------------------------------------------------------------------*/

#include "DeviceAcousticTransceiver.h"
#include "PhysicsBullet.h"
#include "PhysicsBulletInterface.h"
#include "RenderOpenGLInterface.h"
#include "Object.h"

#include <list>
#include <vector>
#include <iostream>
#include <typeinfo>

DeviceAcousticTransceiver::DeviceAcousticTransceiver(PhysicsBullet* p, btRigidBody* b, btTransform t, int collisionFilter, int collisionType, float maxRange) : 
    btBroadphaseAabbCallback (),
    RenderOpenGLInterface()
{
    this->physics = p;
    this->parentBody = b;
    this->localTransform = t;

    // inherited from ContactResultCallback
    this->collisionFilter = collisionFilter;
    this->collisionType = collisionType;

    // create the collision body
    this->maxRange = maxRange;
    collisionShape = new btSphereShape(maxRange);
    
    btScalar mass(0.0);
    btVector3 localInertia(0.0, 0.0, 0.0);
 		
    btTransform startTransform;
    startTransform.setIdentity();
    btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,collisionShape,localInertia);
    collisionBody = new btRigidBody(rbInfo);    

    // make sure parent body has a user pointer correctly set... 
    // I expect this convention to cause troubles...
    // the user pointer should be set to the corresponding Object* pointer (see objects implemenations such as Floor.cpp)
    if (parentBody->getUserPointer() == NULL)
    {
	std::cerr << "The body " << parentBody << " misses a proper user pointer in its collision shape !" << std::endl;
    }

    Reset();
}


DeviceAcousticTransceiver::~DeviceAcousticTransceiver()
{

}

void DeviceAcousticTransceiver::ActionStep ()
{
    if (messagesSent.size() > 0)
    {
	// move the sensor in its place with local transform
	btTransform t = parentBody->getWorldTransform() * localTransform;
	collisionBody->setWorldTransform(t);
		
	// use the engine to make half work
	btVector3 aabbMin,aabbMax;
	collisionBody->getCollisionShape()->getAabb(t, aabbMin, aabbMax);

	// process will be called when a collision is detected
	btBroadphaseInterface* broadphase =  physics->m_dynamicsWorld->getBroadphase();
	broadphase->aabbTest(aabbMin,aabbMax, (*this));
	
	// all messages have been sent
	messagesSent.clear();
    }
}


void DeviceAcousticTransceiver::PerceptionStep ()
{
    // everything is done in action step...
}
    
void DeviceAcousticTransceiver::Reset ()
{
    messagesReceived.clear();
    messagesSent.clear();
}

void DeviceAcousticTransceiver::Draw (RenderOpenGL* r)
{   
    btTransform transform;
    btScalar m[16];
    btMatrix3x3 rot;
    rot.setIdentity();

    transform = parentBody->getWorldTransform();
    
    transform *= localTransform;
    transform.getOpenGLMatrix(m);

    glPushMatrix();     
    glMultMatrixf(m);


    dsRotationMatrix rotmat;
    float pos[3]; pos[0] = 0; pos[1] = 0; pos[2] = 0;       
    dRotation2dsRotationMatrix(0, rotmat);
    r->dsSetColorAlpha (0.9, 0.9, 0.9, 0.3);
    r->dsDrawCylinder((float*) &pos, (float*) &rotmat, 0.1, maxRange);

    // glEnable(GL_COLOR_MATERIAL);
    // glEnable (GL_BLEND);
    // glBlendFunc (GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    // glColor4f(1, 1, 1, 1);
    
    // glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    // glLineWidth(0.5);
    // glutWireSphere(maxRange, 8, 8);

    // glPolygonMode(GL_FRONT, GL_FILL);
    // glDisable(GL_COLOR_MATERIAL);

    glPopMatrix();
}


bool DeviceAcousticTransceiver::process (const btBroadphaseProxy *proxy)
{
    // do not take into account oneself, or parent
    btCollisionObject* detectedObject = (btCollisionObject*)proxy->m_clientObject;
    if (detectedObject == collisionBody) return true;
    if (detectedObject == parentBody) return true;
	
    //only perform raycast if filterMask matches
    btBroadphaseProxy* bph = detectedObject->getBroadphaseHandle();
    bool collides = (bph->m_collisionFilterGroup & this->collisionFilter) != 0;
    collides = collides && (this->collisionType & bph->m_collisionFilterMask);

    if (collides) 
    {
	// We say the acoustic signal is perceived iff the object COM falls within the emission radius...		
	btTransform other = detectedObject->getWorldTransform();
	btTransform self = collisionBody->getWorldTransform();	

        btVector3 diff = other.getOrigin() - self.getOrigin();
        btScalar dist = diff.length();

        if ( dist > maxRange)
        {
	    return true;
        }

	// ok if the object has acoustic sensors, they detect the signal
	Object* o = (Object*) detectedObject->getUserPointer();
	
	for (unsigned int i = 0; i < o->devices.size(); i++)
	{
	    // found the device	
	    if (typeid (*(o->devices[i])) == typeid(*this))
	    {
		// transmit message and exit
		DeviceAcousticTransceiver* at = (DeviceAcousticTransceiver*) o->devices[i];
		
		std::list<Message>::iterator it2 = messagesSent.begin();
		for (; it2 != messagesSent.end(); ++it2)
		{
		    Message m (dist, it2->content);
		    at->messagesReceived.push_back(m);
		}

		// messages processed, exit
		return true;
	    }
	}
    }
    return true;
}


void DeviceAcousticTransceiver::Send (int content)
{
    // this message will be sent in a given radius
    Message m (maxRange, content);
    messagesSent.push_back(m);
}

bool DeviceAcousticTransceiver::Receive (int& content)
{
    if (messagesReceived.empty())
    {	
	return false;
    }
    else
    {
	content = messagesReceived.front().content;
	messagesReceived.pop_front();
    }
}
