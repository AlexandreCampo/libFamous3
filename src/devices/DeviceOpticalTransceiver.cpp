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

// BUGS and FEATURES 
// no occlusion at the moment

#include "DeviceOpticalTransceiver.h"
#include "PhysicsBullet.h"
#include "PhysicsBulletInterface.h"
#include "RenderOpenGLInterface.h"
#include "Simulator.h"

#include <list>
#include <vector>
#include <iostream>
#include <typeinfo>



DeviceOpticalTransceiver::DeviceOpticalTransceiver(PhysicsBullet* p, btRigidBody* b, btTransform t, int collisionFilter, int collisionType, float maxRange) : 
    Device (),
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

void DeviceOpticalTransceiver::AddTransmitter (btVector3 position, btVector3 direction, btScalar range, btScalar angle)
{
    // there is a maximum number of transmitters...
    if (transmitters.size() >= DEVICE_OPTICAL_TRANSCEIVER_MAX_TRANSMITTERS)
    { 
	std::cerr << "The object " << parentBody->getUserPointer() << " is trying to add more optical transmitters than allowed !" << std::endl;
	return;
    }
    // local ray coordinates
    transmitters.push_back (Transmitter (position, direction.normalize(), range, angle));
}


DeviceOpticalTransceiver::~DeviceOpticalTransceiver()
{

}

void DeviceOpticalTransceiver::ActionStep ()
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
	
	// remove all messages that have been sent
	std::list<Message>::iterator it = messagesSent.begin();

	for (auto i = messagesSent.begin(); i != messagesSent.end();)
	{
	    if (i->time < object->simulator->time)
		i = messagesSent.erase(i);
	    else
		++i;
	}
    }
}

void DeviceOpticalTransceiver::PerceptionStep ()
{
    // everything is done in action step...
}
    
void DeviceOpticalTransceiver::Reset ()
{
    messagesReceived.clear();
    messagesSent.clear();
}

void DeviceOpticalTransceiver::Draw (RenderOpenGL* r)
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

    // also draw transmitters
    std::vector<Transmitter>::iterator it = transmitters.begin();
    for (; it != transmitters.end(); ++it)
    {
	// draw the ray
	float pos1[3] = {0,0,0};	
	float pos2[3] = {0,0,0};	
	
	pos1[0] = it->rayFromLocal.x(); pos1[1] = it->rayFromLocal.y(); pos1[2] = it->rayFromLocal.z(); 
	btVector3 colpos = it->rayToLocal;
	pos2[0] = colpos.x(); pos2[1] = colpos.y(); pos2[2] = colpos.z(); 
	
	glLineWidth (1.0);
	r->dsDrawLine (pos1, pos2);
    }

    // r->dsSetColorAlpha (0.9, 0.9, 0.9, 0.3);
    // // r->dsDrawCylinder((float*) &pos, (float*) &rotmat, 0.1, maxRange);

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


bool DeviceOpticalTransceiver::process (const btBroadphaseProxy *proxy)
{
    // do not take into account oneself, or parent
    btCollisionObject* detectedObject = (btCollisionObject*)proxy->m_clientObject;
    if (detectedObject == collisionBody) return true;
    if (detectedObject == parentBody) return true;
	
    //only perform raycast if filterMask matches
    btBroadphaseProxy* bph = detectedObject->getBroadphaseHandle();
    bool collides = (bph->m_collisionFilterGroup & collisionFilter) != 0;
    collides = collides && (collisionType & bph->m_collisionFilterMask);

    if (collides) 
    {
	// We say the optical signal is perceived iff the object COM falls within the emission radius...		
	btTransform other = detectedObject->getWorldTransform();
	btTransform self = collisionBody->getWorldTransform();	

        btVector3 diff = other.getOrigin() - self.getOrigin();
        btScalar dist = diff.length();

        if ( dist > maxRange)
        {
	    return true;
        }

	// a buffer to avoid redundant calculation of in cone message passing
	int insideTransmitterCone [DEVICE_OPTICAL_TRANSCEIVER_MAX_TRANSMITTERS];
	for (int i = 0; i < DEVICE_OPTICAL_TRANSCEIVER_MAX_TRANSMITTERS; i++) insideTransmitterCone[i] = -1;

	// memorize device pointer (speed up if more than one message transmitted)
	DeviceOpticalTransceiver* ot = NULL;
	bool transceiverExists = true;

	// now loop over all messages and see which ones are received (no occlusion in this case ...)
	std::list<Message>::iterator it = messagesSent.begin();
	for (; it != messagesSent.end(); ++it)
	{
	    // check if message must be delivered later
	    if (it->time < object->simulator->time) continue;
	    
	    bool messageTransmitted = true;

	    // a specific transmitter was selected 
	    if (it->transmitter >= 0)
	    {
		// first check if the presence in cone was already tested for this object
		if (insideTransmitterCone[it->transmitter] == -1)
		{
		    btVector3 diffnorm = diff / dist;
		    btMatrix3x3 selfbasis = self.getBasis();
		    btVector3 tr (selfbasis[0].dot(transmitters[it->transmitter].direction), selfbasis[1].dot(transmitters[it->transmitter].direction), selfbasis[2].dot(transmitters[it->transmitter].direction));
		    float cosAngle = (tr).dot(diffnorm);
		    
		    // the object CoM falls inside transmission cone
		    if (cosAngle > transmitters[it->transmitter].cosApertureAngle)
		    {
			insideTransmitterCone[it->transmitter] = 1;
		    }
		    else
		    {
			messageTransmitted = false;
			insideTransmitterCone[it->transmitter] = 0;
		    }
		} 
		else if (insideTransmitterCone[it->transmitter] == 0)
		{
		    messageTransmitted = false;
		}		    
	    }

	    if (messageTransmitted)
	    {
		// set the emitter
		it->emitter = (Object*) parentBody->getUserPointer();

		if (ot == NULL)
		{
		    if (transceiverExists)
		    {
			transceiverExists = false; 

			// ok if the object has optical sensors, they detect the signal
			Object* o = (Object*) detectedObject->getUserPointer();
			
			for (unsigned int i = 0; i < o->devices.size(); i++)
			{
			    // found the device	
			    if (typeid (*(o->devices[i])) == typeid(*this))
			    {
				// transmit message and exit
				ot = (DeviceOpticalTransceiver*) o->devices[i];
				// Message m (dist, it->content, it->time, -1);
				
				ot->messagesReceived.push_back(*it);
				transceiverExists = true;
				break;
			    }
			}
		    }
		}
		else
		{
		    ot->messagesReceived.push_back(*it);
		}
	    }
	}
    }
    return true;
}


void DeviceOpticalTransceiver::Send (int content, float time, int transmitter)
{
    Message m (maxRange, content, time, transmitter);
    messagesSent.push_back(m);
}

void DeviceOpticalTransceiver::SendOmnidirectional (int content, float time)
{
    Message m (maxRange, content, time, -1);
    messagesSent.push_back(m);
}

bool DeviceOpticalTransceiver::Receive (int& content)
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

void DeviceOpticalTransceiver::SetRange (float range)
{
    // change the large range
    maxRange = range;
    btSphereShape* s = static_cast<btSphereShape*> (collisionShape);
    s->setUnscaledRadius (range);

    // go through all transmitters and adjust range
    std::vector<Transmitter>::iterator it = transmitters.begin();
    for (; it != transmitters.end(); ++it)
    {
	it->rayToLocal = it->rayFromLocal + it->direction * range;
    }    
}
