/*----------------------------------------------------------------------------*/
/*    Copyright (C) 2011-2017 Alexandre Campo                                 */
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

#include "DeviceDocker.h"
#include "PhysicsBullet.h"
#include "PhysicsBulletInterface.h"
#include "RenderOpenGLInterface.h"
#include "Object.h"

#include <list>
#include <vector>
#include <iostream>
#include <typeinfo>

DeviceDocker::DeviceDocker(PhysicsBullet* p, btRigidBody* b, float maxDetectionRange, float maxDockingRange, float verticalTolerance, float detectionAngle, int detectionCollisionFilter, int dockingCollisionFilter, int detectionCollisionType, int dockingCollisionType) : 
    btBroadphaseAabbCallback (),
    RenderOSGInterface()
{
    this->physics = p;
    this->parentBody = b;

    // inherited from ContactResultCallback
    this->detectionCollisionFilter = detectionCollisionFilter;
    this->detectionCollisionType = detectionCollisionType;
    this->dockingCollisionFilter = dockingCollisionFilter;
    this->dockingCollisionType = dockingCollisionType;

    // create the collision body
    this->maxDockingRange = maxDockingRange;
    this->maxDetectionRange = maxDetectionRange;
    this->detectionAngle = detectionAngle;
    cosDetectionAngle = cos(detectionAngle);
    this->verticalTolerance = verticalTolerance;

    // collision shape/body for detection of other docker devices
    btScalar mass(0.0);
    btVector3 localInertia(0.0, 0.0, 0.0);

    collisionShape = new btCylinderShapeZ(btVector3(maxDetectionRange, maxDetectionRange, verticalTolerance * 2) / 2.0);
     		
    btTransform startTransform;
    startTransform.setIdentity();
    btDefaultMotionState* motionState = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,motionState,collisionShape,localInertia);
    collisionBody = new btRigidBody(rbInfo);
    collisionBody->setUserPointer(this);
    
    // make sure parent body has a user pointer correctly set... 
    // I expect this convention to cause troubles...
    // the user pointer should be set to the corresponding Object* pointer (see objects implemenations such as Floor.cpp)
    if (parentBody->getUserPointer() == NULL)
    {
	std::cerr << "The body " << parentBody << " misses a proper user pointer in its collision shape !" << std::endl;
    }

    reset();
}


DeviceDocker::~DeviceDocker()
{

}

void DeviceDocker::actionStep ()
{
}


void DeviceDocker::perceptionStep ()
{
    calculated = false;
    detectedPositions.clear();
    closestDevice = NULL;
    closestDeviceDistance = std::numeric_limits<float>::max();
    closestDeviceRelativePosition.setZero();
}
    
void DeviceDocker::reset ()
{
    calculated = false;
    dockable = true;
    detectedPositions.clear();
    closestDeviceDistance = std::numeric_limits<float>::max();

    // release any connected device
    undock();
}

void DeviceDocker::setPosition(btVector3 position)
{
    localTransform.setOrigin(position);
}

btVector3 DeviceDocker::getPosition()
{
    return localTransform.getOrigin();
}

void DeviceDocker::setOrientation(btQuaternion q)
{
    localTransform.setRotation(q);
}

btQuaternion DeviceDocker::getOrientation()
{
    return localTransform.getRotation();
}


btVector3 DeviceDocker::getClosestDockableDevice()
{
    getDockableDevices();
    return closestDeviceRelativePosition;
}

    
std::vector<btVector3>& DeviceDocker::getDockableDevices()
{
    if (!calculated)
    {
	// move the sensor in its place with local transform
	btTransform t = parentBody->getWorldTransform() * localTransform;
	collisionBody->setWorldTransform(t);
	inverseLocalRotation = t.getBasis().transpose();
	
	// use the engine to make half work
	btVector3 aabbMin,aabbMax;
	collisionBody->getCollisionShape()->getAabb(t, aabbMin, aabbMax);
	
	// process will be called when a collision is detected
	btBroadphaseInterface* broadphase =  physics->m_dynamicsWorld->getBroadphase();
	broadphase->aabbTest(aabbMin,aabbMax, (*this));

	// stored results are valid for this time step
	calculated = true;
    }

    return detectedPositions;
}

bool DeviceDocker::dock()
{
    // already docked
    if (docked)
    {
	return false;
    }
    
    getDockableDevices();

    // within docking range
    if (closestDeviceDistance >= maxDockingRange)
    {
	return false;
    }
    
    // only if device accepts to be docked
    if (!closestDevice->dockable)
    {
	return false;
    }

    // and that device is not already docked
    if (closestDevice->docked)
    {
	return false;
    }
    
    // allright, device is within range, not already docked, implement docking...    
    constraint = new btFixedConstraint(*parentBody, *closestDevice->parentBody, localTransform, closestDevice->localTransform);
    physics->m_dynamicsWorld->addConstraint(constraint);

    docked = closestDevice;
    closestDevice->docked = this;
    closestDevice->constraint = constraint;
    
    return true;
}

bool DeviceDocker::undock()
{
    if (docked == NULL)
    {
	return false;
    }

    // undock unilaterally, no matter who initiated docking		
    physics->m_dynamicsWorld->removeConstraint(constraint);
    delete constraint;
    docked->constraint = NULL;
    docked->docked = NULL;
    docked = NULL;
    return true;
}

bool DeviceDocker::process (const btBroadphaseProxy *proxy)
{
    // do not take into account oneself, or parent
    btCollisionObject* detectedObject = (btCollisionObject*)proxy->m_clientObject;
    if (detectedObject == collisionBody) return true;
    if (detectedObject == parentBody) return true;
	
    //only perform tests if filterMask matches
    btBroadphaseProxy* bph = detectedObject->getBroadphaseHandle();
    bool collides = (bph->m_collisionFilterGroup & this->detectionCollisionFilter) != 0;
    collides = collides && (this->detectionCollisionType & bph->m_collisionFilterMask);

    if (collides) 
    {
	// now get the device of the object
	Object* o = (Object*) detectedObject->getUserPointer();
	
	for (auto* device : o->devices)
	{
	    // found the device
	    if (typeid (*device) == typeid (*this))
	    {
		DeviceDocker* d = (DeviceDocker*) device;
		
		if (!d->dockable)
		{
		    return true;
		}
		
		// We say the dockable object is perceived iff the center of device is in the detection shape		
		btTransform other = d->parentBody->getWorldTransform() * d->localTransform;
		btTransform self = collisionBody->getWorldTransform();	
		
		btVector3 diff = other.getOrigin() - self.getOrigin();
		
		btScalar dist = diff.length();
		
		// check for distance first
		if ( dist > maxDetectionRange)
		{
		    return true;
		}

		btVector3 diffnorm = diff / dist;
		btVector3 wdir = collisionBody->getWorldTransform().getBasis().getColumn(0);
    
		float cosAngle = wdir.dot (diffnorm);
    
		// the device falls outside detection cone
		if (cosAngle < cosDetectionAngle)
		{
		    return true;
		}
		    
		// TODO bring back the check when needed ...		
		// now check if inside the cylinder
//		btVector3 localOther = self.inverse() * diff;
//		if (fabs(localOther.getZ()) > verticalTolerance)
		// if (fabs(diff.z()) > verticalTolerance)
		// {		    
		//     return true;
		// }
	
		
		// tranform diff vector back into device local frame
		btVector3 localDiff = inverseLocalRotation * diff;
		
		// found a dockable device, record ptr
		detectedPositions.push_back(localDiff);
		
		// record the closest device
		if (dist < closestDeviceDistance)
		{
		    closestDeviceDistance = dist;
		    closestDeviceRelativePosition = localDiff;
		    closestDevice = d;
		}
		
	    }
	}
    }    

    return true;
}

void DeviceDocker::setDockable(bool d)
{
    dockable = d;
}

bool DeviceDocker::isDockable()
{
    return dockable;
}

void DeviceDocker::setDrawable(bool d)
{
    drawable = d;
    RenderOSGInterface::transform->setNodeMask(d);
}

bool DeviceDocker::isDrawable()
{
    return drawable;
}

void DeviceDocker::registerService (RenderOSG* r)
{
    RenderOSGInterface::registerService (r);

    if (!deviceDockerNode)
    {
    	osg::ref_ptr<osg::Geode> geode; 
    	osg::ref_ptr<osg::Cone> cone; 
    	osg::ref_ptr<osg::ShapeDrawable> coneDrawable;

	// arbitrary aperture angle... not yet known
	float angle = 20.0 * M_PI / 180.0;
	float radius = tan(angle) * maxDockingRange;
    	cone = new osg::Cone(osg::Vec3(0,0,-maxDockingRange*3.0/4.0), radius, maxDockingRange); 
    	coneDrawable = new osg::ShapeDrawable(cone); 
    	geode = new osg::Geode; 
    	geode->addDrawable(coneDrawable); 
    	deviceDockerNode = geode;
    }
    
    // one transform for spatial position, will be adjusted to data from physics engine
    RenderOSGInterface::transform = new osg::MatrixTransform();

    // setup rendering
    
    // rotate, translate, scale to device location
    osg::ref_ptr<osg::PositionAttitudeTransform> t = new osg::PositionAttitudeTransform();
    btVector3 p = localTransform.getOrigin();
    t->setPosition(osg::Vec3d(p.x(), p.y(), p.z()));

    btQuaternion bq;
    localTransform.getBasis().getRotation(bq);

    float angle = bq.getAngle();
    btVector3 axis = bq.getAxis();
    
    osg::Quat q = osg::Quat (angle, osg::Vec3(axis.x(), axis.y(), axis.z())); 
    q = osg::Quat (M_PI/4, osg::Vec3(1,1,1)); 
    t->setAttitude(q);

    osg::Quat q1 = osg::Quat( -M_PI/2.0, osg::Vec3(0,1,0) );
    osg::Quat q2 = osg::Quat(angle, osg::Vec3(axis.x(), axis.y(), axis.z()));	
    t->setAttitude(q1 * q2);

    
    t->addChild (deviceDockerNode);
    RenderOSGInterface::transform->addChild (t);
    renderOSG->root->addChild(RenderOSGInterface::transform);
    
    // disable rendering by default
    RenderOSGInterface::transform->setNodeMask(0);
    
    osg::ref_ptr<osg::Material> mat = new osg::Material;
    mat->setDiffuse (osg::Material::FRONT_AND_BACK, osg::Vec4(0.5, 0.5, 1, 0.2));
    deviceDockerNode->getOrCreateStateSet()->setAttributeAndModes(mat, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);	    
}

void DeviceDocker::draw (RenderOSG* r)
{
    if (drawable)
    {
	btScalar ogl[16];
	btTransform t = parentBody->getWorldTransform();
	t.getOpenGLMatrix( ogl );
	osg::Matrix m(ogl);
	RenderOSGInterface::transform->setMatrix (m);
    }
}
