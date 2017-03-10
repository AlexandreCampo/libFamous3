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

using namespace std;

DeviceOpticalTransceiver::DeviceOpticalTransceiver(PhysicsBullet* p, btRigidBody* b, btTransform t, int collisionFilter, int collisionType, float maxRange) : 
    Device (),
    btBroadphaseAabbCallback ()
{
    this->physics = p;
    this->body = b;
    this->transform = t;

    receiveOmnidirectional = false;
    drawable = false;
    
    // inherited from callback
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
    if (body->getUserPointer() == NULL)
    {
	std::cerr << "The body " << body << " misses a proper user pointer in its collision shape !" << std::endl;
    }

    Reset();
}

void DeviceOpticalTransceiver::AddTransmitter (btVector3 position, btVector3 direction, btScalar range, btScalar angle)
{
    transmitters.push_back (Transmitter (position, direction, range, angle));

    if (renderOSG)
    {
	direction.normalize();
	
	// rotate, translate, scale to device location
	osg::ref_ptr<osg::PositionAttitudeTransform> t = new osg::PositionAttitudeTransform();
	btVector3 p = transform.getOrigin() + position;
	t->setPosition(osg::Vec3d(p.x(), p.y(), p.z()));
	const btMatrix3x3& rot = transform.getBasis();
	btVector3 rdir = rot.getColumn(0);
	btVector3 raxis = rdir.cross(direction);

	// direction parallel to x axis, can only be a rotation around y
	if (raxis.fuzzyZero())
	    raxis = btVector3(0,1,0);
	
	float cosAngle = rdir.dot(direction);
	osg::Quat q1 = osg::Quat( -M_PI/2.0, osg::Vec3(0,1,0) );
	osg::Quat q2 = osg::Quat(acos(cosAngle), osg::Vec3(raxis.x(), raxis.y(), raxis.z()));	
	t->setAttitude(q1 * q2);
	
	t->addChild (deviceOpticalTransceiverNode);
	RenderOSGInterface::transform->addChild (t);
	renderOSG->root->addChild(RenderOSGInterface::transform);

	// disable rendering by default
	RenderOSGInterface::transform->setNodeMask(0);
	
	osg::ref_ptr<osg::Material> mat = new osg::Material;
	mat->setDiffuse (osg::Material::FRONT_AND_BACK, osg::Vec4(0.5, 0.5, 1, 0.2));
	deviceOpticalTransceiverNode->getOrCreateStateSet()->setAttributeAndModes(mat, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);	
    }
}

void DeviceOpticalTransceiver::AddReceiver (btVector3 position, btVector3 direction, btScalar range, btScalar angle)
{
    receivers.push_back (Receiver (position, direction, range, angle));
}


DeviceOpticalTransceiver::~DeviceOpticalTransceiver()
{

}

void DeviceOpticalTransceiver::ActionStep ()
{
    if (messagesSent.size() > 0)
    {
	// move the sensor in its place with local transform
	btTransform t = body->getWorldTransform() * transform;
	collisionBody->setWorldTransform(t);
	
	// use the engine to make half work
	btVector3 aabbMin,aabbMax;
	collisionBody->getCollisionShape()->getAabb(t, aabbMin, aabbMax);
	
	// process will be called when a collision is detected
	btBroadphaseInterface* broadphase =  physics->m_dynamicsWorld->getBroadphase();
	broadphase->aabbTest(aabbMin,aabbMax, (*this));
	
	// remove all messages that have been sent
	messagesSent.clear();
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

bool DeviceOpticalTransceiver::processOmniToOmni(Message& m, btCollisionObject* detectedObject, DeviceOpticalTransceiver* od)
{
    // check that both devices are in range
    btVector3 otherPos = detectedObject->getWorldTransform() * od->transform.getOrigin();
    btVector3 selfPos = collisionBody->getWorldTransform().getOrigin();	
    
    btVector3 diff = otherPos - selfPos;
    btScalar dist = diff.length();
    
    // not in range
    if ( dist > maxRange) return true;

    m.direction = diff / dist;
    m.distance = dist;
    od->messagesReceived.push_back(m);

    return true;
}

bool DeviceOpticalTransceiver::processTransmitterToOmni(Message& m, const Transmitter& transmitter, btCollisionObject* detectedObject, DeviceOpticalTransceiver* od)
{
    // check that device is within transmitter cone
    btVector3 otherPos = detectedObject->getWorldTransform() * od->transform.getOrigin();
    btVector3 selfPos = collisionBody->getWorldTransform() * transmitter.position;	
    
    btVector3 diff = otherPos - selfPos;
    btScalar dist = diff.length();
    
    if ( dist > maxRange) return true;

    btVector3 diffnorm = diff / dist;
    btVector3 wdir = collisionBody->getWorldTransform().getBasis() * transmitter.direction;
    
    float cosAngle = wdir.dot (diffnorm);
    
    // the device falls inside transmission cone
    if (cosAngle >= transmitter.cosApertureAngle)
    {
	m.direction = diffnorm;
	m.distance = dist;
	od->messagesReceived.push_back(m);
    }    
    return true;
}

bool DeviceOpticalTransceiver::processOmniToReceivers(Message& m, btCollisionObject* detectedObject, DeviceOpticalTransceiver* od)
{
    const btTransform& otherTr = detectedObject->getWorldTransform() * od->transform;
    
    // loop over all receivers
    int r = 0;
    for (auto receiver : od->receivers)
    {
	// check that receiver is in range
	btVector3 otherPos = otherTr * receiver.position;
	const btVector3& selfPos = collisionBody->getWorldTransform().getOrigin();	
	
	btVector3 diff = selfPos - otherPos;
	btScalar dist = diff.length();
	
	// not in range, skip this receiver
	if ( dist > maxRange) continue;
	
	// check that self device is in receiver cone
	btVector3 wodir = otherTr.getBasis() * receiver.direction;
	btVector3 diffnorm = diff / dist;
	
	float cosAngleReceiver = wodir.dot (diffnorm);
	
	// transmitter is inside receiver cone
	if (cosAngleReceiver >= receiver.cosApertureAngle)
	{
	    m.receiver = r;
	    m.direction = receiver.direction;
	    m.distance = dist;
	    od->messagesReceived.push_back(m);
	}
	r++;
    }		    

    return true;
}

bool DeviceOpticalTransceiver::processTransmitterToReceivers(Message& m, const Transmitter& transmitter, btCollisionObject* detectedObject, DeviceOpticalTransceiver* od)
{
    const btTransform& otherTr = detectedObject->getWorldTransform() * od->transform;
    const btTransform& selfTr = collisionBody->getWorldTransform();

    // loop over all receivers
    int r = 0;
    for (auto receiver : od->receivers)
    {
	// check that receiver is in transmitter cone
	btVector3 otherPos = otherTr * receiver.position;
	btVector3 selfPos = selfTr * transmitter.position;	
	
	btVector3 diff = otherPos - selfPos;
	btScalar dist = diff.length();
	
	// not in range, skip this receiver
	if ( dist > maxRange) continue;
	
	btVector3 diffnorm = diff / dist;

	btVector3 wdir = selfTr.getBasis() * transmitter.direction;
	float cosAngleTransmitter = wdir.dot (diffnorm);
	
	// the receiver falls inside transmission cone
	if (cosAngleTransmitter >= transmitter.cosApertureAngle)
	{			    
	    // check that transmitter is in receiver cone
	    btVector3 wodir = otherTr.getBasis() * receiver.direction;
	    float cosAngleReceiver = wodir.dot (-diffnorm);
	    
	    // transmitter is inside receiver cone
	    if (cosAngleReceiver >= receiver.cosApertureAngle)
	    {
		m.receiver = r;
		m.direction = receiver.direction;
		m.distance = dist;
		od->messagesReceived.push_back(m);
	    }
	}
	r++;
    }		    
    return true;
}


bool DeviceOpticalTransceiver::process (const btBroadphaseProxy *proxy)
{
    // do not take into account oneself, or parent
    btCollisionObject* detectedObject = (btCollisionObject*)proxy->m_clientObject;
    if (detectedObject == collisionBody) return true;
    if (detectedObject == body) return true;
	
    //only manage messages if filterMask matches
    btBroadphaseProxy* bph = detectedObject->getBroadphaseHandle();
    bool collides = (bph->m_collisionFilterGroup & collisionFilter) != 0;
    collides = collides && (collisionType & bph->m_collisionFilterMask);

    // found one object within communication range
    if (collides) 
    {
	// check if colliding object has an optical device
	Object* o = (Object*) detectedObject->getUserPointer();
	DeviceOpticalTransceiver* od = NULL;
	for (auto d : o->devices)
	{
	    if (typeid (*d) == typeid (*this))
	    {
		od = (DeviceOpticalTransceiver*) d;
		break;
	    }
	}

	// no optical device, no transmission
	if (od == NULL) return true;

	for (auto message : messagesSent)
	{
	    // a specific transmitter was selected 
	    if (message.transmitter >= 0)
	    {
		const Transmitter& transmitter = transmitters[message.transmitter];
		
		// other device receives from any direction
		if (od->receiveOmnidirectional)
		{
		    return processTransmitterToOmni(message, transmitter, detectedObject, od);
		}		
		// other device uses receivers
		else
		{
		    return processTransmitterToReceivers(message, transmitter, detectedObject, od);
		}		
	    }
	    // send in all directions
	    else
	    {		
		// other device receives from any direction
		if (od->receiveOmnidirectional)
		{
		    return processOmniToOmni(message, detectedObject, od);
		    
		}		
		// other device uses receivers
		else
		{
		    return processOmniToReceivers(message, detectedObject, od);
		}			    		
	    }
	}
    }
    return true;
}


void DeviceOpticalTransceiver::Send (int content, int transmitter)
{
    Message m (maxRange, content, transmitter);
    messagesSent.push_back(m);
}

bool DeviceOpticalTransceiver::Receive (Message& m)
{
    if (messagesReceived.empty())
    {	
	return false;
    }
    else
    {
	m = messagesReceived.front();
	messagesReceived.pop_front();
    }
}

void DeviceOpticalTransceiver::SetRange (float range)
{
    // change the large range
    maxRange = range;
    btSphereShape* s = static_cast<btSphereShape*> (collisionShape);
    s->setUnscaledRadius (range);
}

void DeviceOpticalTransceiver::SetReceiveOmnidirectional(bool omni)
{
    receiveOmnidirectional = omni;
}

void DeviceOpticalTransceiver::SetDrawable(bool d)
{
    drawable = d;
    RenderOSGInterface::transform->setNodeMask(d);
}

bool DeviceOpticalTransceiver::IsDrawable()
{
    return drawable;
}

void DeviceOpticalTransceiver::Register (RenderOSG* r)
{
    RenderOSGInterface::Register (r);

    if (!deviceOpticalTransceiverNode)
    {
    	osg::ref_ptr<osg::Geode> geode; 
    	osg::ref_ptr<osg::Cone> cone; 
    	osg::ref_ptr<osg::ShapeDrawable> coneDrawable;

	// arbitrary aperture angle... not yet known
	float angle = 20.0 * M_PI / 180.0;
	float radius = tan(angle) * maxRange;
    	cone = new osg::Cone(osg::Vec3(0,0,-maxRange*3.0/4.0),radius,maxRange); 
    	coneDrawable = new osg::ShapeDrawable(cone); 
    	geode = new osg::Geode; 
    	geode->addDrawable(coneDrawable); 
    	deviceOpticalTransceiverNode = geode;
    }
    
    // one transform for spatial position, will be adjusted to data from physics engine
    RenderOSGInterface::transform = new osg::MatrixTransform();
}

void DeviceOpticalTransceiver::Draw (RenderOSG* r)
{
    if (drawable)
    {
	btScalar ogl[16];
	btTransform t = body->getCenterOfMassTransform();
	t.getOpenGLMatrix( ogl );
	osg::Matrix m(ogl);
	RenderOSGInterface::transform->setMatrix (m);
    }
}

