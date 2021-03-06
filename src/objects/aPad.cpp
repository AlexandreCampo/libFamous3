/*----------------------------------------------------------------------------*/
/*    Copyright (C) 2011-2015 Alexandre Campo                                 */
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

#include "aPad.h"
#include "Simulator.h"

#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/Material>

#include <iostream>

using namespace std;

aPad::aPad() :
    Object(),
    PhysicsBulletInterface(),
    RenderOSGInterface(),
    WaterVolumeInterface()
{
    // define collision type
    setCollisionType (1 << 2);
    setCollisionFilter (0x7FFFFFFF);

    acousticCollisionFilter = (1 << 2); // TODO we should probably not include walls ....
    acousticCollisionType = (1 << 3);

    colr = 1.0;
    colg = 1.0;
    colb = 54.0 / 254.0;
    cola = 1.0;
} 


aPad::~aPad()
{

}

void aPad::addDevices () 
{
    // simplified actuator : left+right for rotation, and central for omnidirectional
    propellerLeft = new DevicePropeller (physicsBullet, body, 7);
    propellerLeft->setPosition(centerOfVolume + btVector3( 0, -dimensions[1] / 2.0, 0 ));
    propellerLeft->setOrientation(btQuaternion( 0, 0, 0 ));
    if (renderOSG)
    {
	propellerLeft->registerService(renderOSG);
	propellerLeft->setDrawable(false);
    }
    this->add(propellerLeft);
    
    propellerRight = new DevicePropeller (physicsBullet, body, 7);
    propellerRight->setPosition(centerOfVolume + btVector3( 0, dimensions[1] / 2.0, 0 ));
    propellerRight->setOrientation(btQuaternion( 0, 0, 0 ));
    if (renderOSG)
    {
	propellerRight->registerService(renderOSG);	
	propellerRight->setDrawable(false);
    }
    this->add(propellerRight);
    
    propellerCentral = new DevicePropeller (physicsBullet, body, 7);
    propellerCentral->setPosition(centerOfVolume);
    propellerCentral->setOrientation(btQuaternion( 0, 0, 0 ));
    if (renderOSG)
    {
	propellerCentral->registerService(renderOSG);	
	propellerCentral->setDrawable(false);
    }
    this->add(propellerCentral);


// 4 ballasts
    for (int i = 0; i < 4; i++)
    {
    	float neutralVolume = (1.0 / body->getInvMass()) / waterVolume->density;
    	neutralVolume /= 2.0;
    	float ballastVolume = (0.1 * 0.1 * 0.1);

    	btVector3 relpos = btVector3(dimensions[0], dimensions[1], 0) / 2.0;
    	btQuaternion rotation (btVector3(0,0,1), M_PI / 2.0 * i);

    	DeviceBallast* ballast = new DeviceBallast (centerOfVolume + quatRotate (rotation, relpos), physicsBullet, waterVolume, body, neutralVolume - ballastVolume / 2.0, neutralVolume + ballastVolume / 2.0, 2.0);

    	ballasts.push_back(ballast);
    	this->add (ballast);
    }

    // 4 docking systems
    int cfDetect = this->collisionType;
    int cfDocking = this->collisionType;
    int ctDetect = (1 << 6);
    int ctDocking = (1 << 7);

    for (int i = 0; i < 4; i++)
    {
	DeviceDocker* docker = new DeviceDocker (physicsBullet, body, 1.5, 0.15, 0.1, M_PI/2.0, cfDetect, ctDetect, cfDocking, ctDocking);
	
	// 15 cm off the main body to avoid collisions between docked body and parent body
    	btVector3 relpos = btVector3(dimensions[0] / 2.0 + 0.15, 0, 0);
    	btQuaternion rotation (btVector3(0,0,1), M_PI / 2.0 * i);
	docker->setPosition(centerOfVolume + quatRotate (rotation, relpos));
	docker->setOrientation(btQuaternion (btVector3(0,0,1), M_PI / 2.0 * i));

	if (renderOSG)
	{
	    docker->registerService(renderOSG);	
	}
	
	dockers.push_back(docker);
	add(docker);
    }
        
    acoustic = new DeviceAcousticTransceiver (physicsBullet, body, acousticCollisionFilter, acousticCollisionType, 0.7);
    acoustic->setPosition(centerOfVolume);
    acoustic->setOrientation(btQuaternion( 0, 0, 0 ));
    this->add (acoustic);
}

void aPad::draw (RenderOSG* r)
{
    btScalar ogl[16];
    btTransform t = body->getCenterOfMassTransform() * principalTransformInverse;
    t.getOpenGLMatrix( ogl );
    osg::Matrix m(ogl);
    RenderOSGInterface::transform->setMatrix (m);

    osg::ref_ptr<osg::Material> mat = new osg::Material;
    mat->setDiffuse (osg::Material::FRONT_AND_BACK, osg::Vec4(colr, colg, colb, cola));
    aPadNode->getOrCreateStateSet()->setAttributeAndModes(mat, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
}

void aPad::setMeshFilename (std::string filename)
{
    meshFilename = filename;
}


void aPad::registerService (RenderOSG* r)
{
    RenderOSGInterface::registerService (r);
    
    if (meshFilename.empty())
    {
	cout << "No 3d model defined for OSG Render..." << endl;
	return;
    }

    if (!aPadNode) aPadNode = osgDB::readNodeFile(meshFilename);
//    if (!aPadNode) aPadNode = osgDB::readNodeFile("aPad.3ds.(0.01,0.01,0.01).scale");
    if (!aPadNode)
    {
        std::cout << "Could not load aPad model." << std::endl;
    }

    // one transform for spatial position, will be adjuted to data from physics engine
    RenderOSGInterface::transform = new osg::MatrixTransform();
    
    // one transform for scaling model
    osg::ref_ptr<osg::PositionAttitudeTransform> t = new osg::PositionAttitudeTransform();
    
//    t->setScale(osg::Vec3(1, 1, 1));

    RenderOSGInterface::transform->addChild (t);
    t->addChild (aPadNode);
    r->root->addChild(RenderOSGInterface::transform);

    // text overlay for robot
    text = r->createText(osg::Vec3(-dimensions[0], 0.0f, dimensions[2]), "", dimensions[0]/2);
    textGeode = new osg::Geode;
    textGeode->addDrawable( text );
    textGeode->setNodeMask(0);

    RenderOSGInterface::transform->addChild (textGeode);    
}

void aPad::setTextDrawable(bool d)
{
    if (renderOSG)
    {
	textGeode->setNodeMask(d);
    }
}
    
void aPad::setText(std::string s)
{
    if (renderOSG)
    {
	text->setText(s);
    }
}

void aPad::setTextColor(float r, float g, float b, float a)
{
    if (renderOSG)
    {
	text->setColor(osg::Vec4(r, g, b, a));
    }
}

void aPad::unregisterService (RenderOSG* r)
{
    RenderOSGInterface::unregisterService(r);
}

void aPad::registerService (WaterVolume* w)
{
    WaterVolumeInterface::registerService(w);
    
    // update quadratic drag with new fluid density
    setDragQuadraticCoefficients(m_linearQuadraticDrag, m_angularQuadraticDrag, w->density);
}

void aPad::unregisterService (WaterVolume* w)
{
    WaterVolumeInterface::unregisterService(w);
}

void aPad::setColor (float r, float g, float b, float a)
{
    this->colr = r;
    this->colg = g;
    this->colb = b;
    this->cola = a;

    if (renderOSG)
    {
	osg::ref_ptr<osg::Material> mat = new osg::Material;
	mat->setDiffuse (osg::Material::FRONT_AND_BACK, osg::Vec4(colr, colg, colb, cola));
	RenderOSGInterface::transform->getOrCreateStateSet()->setAttributeAndModes(mat, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
    }    
}

void aPad::registerService (PhysicsBullet* p)
{
    PhysicsBulletInterface::registerService(p);
    setTimestep(p->getTimestep());
    
    dimensions[0] = 0.7;
    dimensions[1] = 0.7;
    dimensions[2] = 0.35;
    mass = 35.0;

    btCompoundShape* cshape = new btCompoundShape(false);
    btBoxShape* boxShape = new btBoxShape(btVector3(dimensions[0], dimensions[1], dimensions[2]) / 2.0);
    principalTransform.setIdentity();
    principalTransformInverse = principalTransform.inverse();
    
    cshape->addChildShape(principalTransformInverse, boxShape);
       
    cshape->recalculateLocalAabb();
    p->m_collisionShapes.push_back(cshape);
    shape = cshape;
    
    shape->calculateLocalInertia(mass,m_inertia);    
    centerOfVolume = principalTransformInverse.getOrigin();
    
    btDefaultMotionState* myMotionState = new btDefaultMotionState(principalTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,m_inertia);
    rbInfo.m_friction = 0.5;    
    rbInfo.m_linearDamping = 0.05; 
    rbInfo.m_angularDamping = 0.05;

    body = new btRigidBody(rbInfo);           
    body->setUserPointer((void*) static_cast<Object*>(this));
    body->setMassProps (mass, m_inertia);

    setDragCoefficients(mass * btVector3( 0.1, 0.1, 0.5), mass * btVector3( 0.3, 0.3, 0.3));
    setDragQuadraticCoefficients(btVector3(0,0,0), btVector3(0,0,0), 1.0);
    setAddedMass (btVector3 (0,0,0), btVector3 (0,0,0));
    body->setActivationState(DISABLE_DEACTIVATION);

    p->m_dynamicsWorld->addRigidBody(body, collisionType, collisionFilter);    
}

void aPad::unregisterService (PhysicsBullet* p)
{
    PhysicsBulletInterface::unregisterService(p);
}


void aPad::setPosition (btVector3 p)
{
    btTransform t = body->getCenterOfMassTransform();
    t.setOrigin(p);
    body->setCenterOfMassTransform(t);
    
    body->setLinearVelocity(btVector3(0,0,0));
    body->setAngularVelocity(btVector3(0,0,0));
}

btVector3 aPad::getPosition ()
{
    return body->getCenterOfMassTransform().getOrigin();
}


void aPad::setRotation (btQuaternion q)
{
    btTransform t = body->getCenterOfMassTransform();
    t.setRotation(q);
    body->setCenterOfMassTransform(t);
    
    body->setLinearVelocity(btVector3(0,0,0));
    body->setAngularVelocity(btVector3(0,0,0));
}

btQuaternion aPad::getRotation (btQuaternion q)
{
    return body->getOrientation();
}


float aPad::getEmissionRange()
{
    return acoustic->maxRange;
}

void aPad::setEmissionRange(float r)
{
    // at the moment it is easier (but slower) to destroy the device and create a new one
    PhysicsBullet* p = acoustic->physics;

    btTransform t = acoustic->localTransform;
    
    this->remove (acoustic);
    delete (acoustic);
	
    acoustic = new DeviceAcousticTransceiver (p, body, acousticCollisionFilter, acousticCollisionType, r);
    acoustic->localTransform = t;
    this->add (acoustic);
}

void aPad::step ()
{
    // custom underwater physics
    const btMatrix3x3& worldBasis = body->getWorldTransform().getBasis();

    btVector3 localLinearVelocity = worldBasis.transpose() * body->getLinearVelocity();
    btVector3 localDragForce = localLinearVelocity * m_linearDrag;
    localDragForce += m_linearQuadraticDrag * localLinearVelocity.absolute() * localLinearVelocity;
    btVector3 dragForce = worldBasis * localDragForce;

    btVector3 angularVelocity = body->getAngularVelocity();
    btVector3 dragTorque = m_angularDrag * angularVelocity;
    dragTorque += m_angularQuadraticDrag * angularVelocity.absolute() * angularVelocity;
    
    body->applyCentralForce (dragForce);	
    body->applyTorque (dragTorque);
}


// the tensor is symmetric
// if the body has 3 planes of symmetry, the tensor becomes diagonal
// since this assumption is made with inertia tensor, we also use it here...
void aPad::setAddedMass (const btVector3& linear, const btVector3& angular)
{
    m_linearAddedMass = linear;
    m_angularAddedMass = angular;
    updateInertiaTensor ();
}

// drag coefficients must be set as 
// linear part : area exposed * coefficient
// angular part : coefficient only
void aPad::setDragCoefficients (const btVector3& linear, const btVector3& angular)
{
    m_linearDrag = -linear;
    m_angularDrag = -angular;
}

void aPad::setDragQuadraticCoefficients (const btVector3& qlinear, const btVector3& qangular, float fluidDensity)
{
    m_linearQuadraticDrag = -qlinear;
    m_angularQuadraticDrag = -qangular;
}


void aPad::updateInertiaTensor ()
{
    body->setMassProps (mass, m_inertia);
    body->updateInertiaTensor();
}

