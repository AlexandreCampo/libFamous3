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

#include "aMussel.h"
#include "Simulator.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"

#include <osg/MatrixTransform>
#include <osg/Material>


#include <iostream>

aMussel::aMussel() :
    Object(),
    PhysicsBulletInterface(),
    WaterVolumeInterface(),
    RenderOSGInterface(),
    EnergyManagerInterface (1.0, 1.0)
{

    colr = 1.0;
    colg = 0.0;
    colb = 0.0;
    cola = 1.0;

    // define collision type
    SetCollisionType (1 << 2);
    SetCollisionFilter (0x7FFFFFFF);
}
 


aMussel::~aMussel()
{
}

void aMussel::AddDevices()
{
    // btVector3 leftPos(0, -dimensions[1] * 1.0, 0);
    // btVector3 rightPos(0, +dimensions[1] * 1.0, 0);
    // propellers = new DevicePropellers(body, leftPos, rightPos, 0.1 * scalingFactor);
    // Add (propellers);

    float neutralVolume = (1.0 / body->getInvMass()) / waterVolume->density;
    float ballastVolume = (3.141592564 * 0.07 * 0.07 * 0.2 * physicsBullet->scalingFactor * physicsBullet->scalingFactor * physicsBullet->scalingFactor);
//    btVector3 centerOfVolume = principalTransform.inverse().getOrigin();

    ballast = new DeviceBallast (centerOfVolume, physicsBullet, waterVolume, body, neutralVolume - ballastVolume / 2.0, neutralVolume + ballastVolume / 2.0, 2.0);
    Add (ballast);
    
    btTransform t2;
    t2.setIdentity();    
    int cf2 = this->collisionType;
    int ct2 = (1 << 3);
    acoustic = new DeviceAcousticTransceiver (physicsBullet, body, t2, cf2, ct2, 0.9 * physicsBullet->scalingFactor);    
    Add (acoustic);

    t2.setIdentity();    
    int cf4 = this->collisionType;
    int ct4 = (1 << 4);
    float range = 0.2 * physicsBullet->scalingFactor;    
    optical = new DeviceOpticalTransceiver (physicsBullet, body, t2, cf4, ct4, range);    
    Add (optical);

    // add directional optical transceivers
//    float addedrange = 0.005 * physicsBullet->scalingFactor;
    float a = 30.0 * M_PI / 180.0;

    // top
    btVector3 pos = btVector3 (0.0, 0.0, 0.0);
    btVector3 dir = btVector3 (0, 0, 1);   
    optical->AddTransmitter (pos, dir, range, 30.0 * M_PI / 180.0);

    // bottom
    pos = btVector3 (0.0, 0.0, 0.0);
    dir = btVector3 (0, 0, -1);   
    optical->AddTransmitter (pos, dir, range, 30.0 * M_PI / 180.0);


    // adding ray sensors for proximity
    int cf3 = 0x7FFFFFFF; 
    int ct3 = this->collisionType; 
    a = 15.0 * M_PI / 180.0;
    float px = cos (a) * cos (a);
    float pyz = sin (a);
    
    // ray sensors may not detect objects that go inside the other object... hence I add some distance.
    float addedrange = 0.005 * physicsBullet->scalingFactor;
    
    range = 0.3 * physicsBullet->scalingFactor + addedrange;
        
    pos = btVector3 (0.0, 0.0, 0.05 * physicsBullet->scalingFactor - addedrange);
    dir = btVector3 (0.0, 0.0, 1.0);   
    rayTop = new DeviceRayCast (physicsBullet, body, pos, dir, range, cf3, ct3);  
    Add (rayTop);
    
    pos = btVector3 (0.0, 0.0, -0.05 * physicsBullet->scalingFactor + addedrange);
    dir = btVector3 (0.0, 0.0, -1.0);   
    rayBottom = new DeviceRayCast (physicsBullet, body, pos, dir, range, cf3, ct3);  
    Add (rayBottom);   

    networker = new DeviceNetworker ();    
    Add (networker);
    
}

void aMussel::Draw (RenderOSG* r)
{
    btScalar ogl[16];
    btTransform t = body->getCenterOfMassTransform() * principalTransform.inverse();
    t.getOpenGLMatrix( ogl );
    osg::Matrix m(ogl);
    transform->setMatrix (m);

    osg::ref_ptr<osg::Material> mat = new osg::Material;
    mat->setDiffuse (osg::Material::FRONT_AND_BACK, osg::Vec4(colr, colg, colb, cola));
    aMusselNode->getOrCreateStateSet()->setAttributeAndModes(mat, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);

    
    
}

void aMussel::Register (PhysicsBullet* p)
{
    PhysicsBulletInterface::Register(p);
    
    SetTimeStep(p->GetTimeStep());
//    p->SetCustomPhysics(true);

    dimensions[0] = 0.2 * physicsBullet->scalingFactor; // diameter (x)
    dimensions[1] = dimensions[0]; // diameter (y)
    dimensions[2] = 0.75 * physicsBullet->scalingFactor; // height (z)

    mass = 5;
//    btScalar waterDensity (1000.0 / (physicsBullet->scalingFactor * physicsBullet->scalingFactor * physicsBullet->scalingFactor));
    btScalar volume = M_PI * dimensions[0] * dimensions[0] * dimensions[1];
    btVector3 localInertia(0.0, 0.0, 0.0);
//    btVector3 linearDrag (0.25, 0.3, 0.9);

    btScalar linearDamping = 0.0;
    btScalar angularDamping = 0.0;

    // we create a first, detailed compound shape in order to calculate the correct inertia tensor
    btCompoundShape* cshape = new btCompoundShape(false);

    float dim1 = dimensions[2] * 0.75; 
    float dim2 = dimensions[2] - dim1; 
    btCylinderShapeZ* cylinderShapeA = new btCylinderShapeZ(btVector3(dimensions[0], dimensions[1], dim1) / 2.0);
    btTransform localTransform;
    localTransform.setIdentity();
    localTransform.setOrigin(btVector3(0, 0, dim2 + dim1 / 2.0 - dimensions[2] / 2.0));
    cshape->addChildShape(localTransform,cylinderShapeA);

    btCylinderShapeZ* cylinderShapeB = new btCylinderShapeZ(btVector3(dimensions[0], dimensions[1], dimensions[2] * 0.25) / 2.0);
    localTransform.setIdentity();
    localTransform.setOrigin (btVector3(0, 0, dim2 / 2.0 - dimensions[2] / 2.0));
    cshape->addChildShape(localTransform,cylinderShapeB);

    btScalar* masses = new btScalar [2];
    masses[0] = 0.2 * mass;
    masses[1] = 0.8 * mass;

    cshape->calculatePrincipalAxisTransform (masses, principalTransform, m_inertia);
    centerOfVolume = principalTransform.inverse().getOrigin();

    // we can delete previous shapes
    delete cshape;
    delete cylinderShapeA;
    delete cylinderShapeB;
    delete [] masses;

    
    // now the real shape used, a simplified compound shape that will be used for collisions
    // create a new compound with world transform/center of mass properly aligned with the principal axis   
    cshape = new btCompoundShape(false); 
    btCylinderShapeZ* cylinderShape = new btCylinderShapeZ(btVector3(dimensions[0], dimensions[1], dimensions[2]) / 2.0);
    
    localTransform.setIdentity();
    btTransform newLocalTransform = principalTransform.inverse() * localTransform;
    cshape->addChildShape(newLocalTransform,cylinderShape);
       
    cshape->recalculateLocalAabb();
    p->m_collisionShapes.push_back(cshape);
    shape = cshape;
   
    btDefaultMotionState* myMotionState = new btDefaultMotionState(principalTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,m_inertia);
    rbInfo.m_friction = 0.05;    
    rbInfo.m_linearDamping = linearDamping; 
    rbInfo.m_angularDamping = angularDamping; 

    body = new btRigidBody(rbInfo);           
    body->setUserPointer((void*) static_cast<Object*>(this));
    body->setMassProps (mass, m_inertia);

    setDragCoefficients(btVector3(0,0,0), btVector3(0,0,0));
    setDragQuadraticCoefficients(btVector3(0,0,0), btVector3(0,0,0), 1.0);
    setAddedMass (btVector3 (0,0,0), btVector3 (0,0,0));
    body->setActivationState(DISABLE_DEACTIVATION);

    p->m_dynamicsWorld->addRigidBody(body, collisionType, collisionFilter);
}

void aMussel::Unregister (PhysicsBullet* p)
{
    PhysicsBulletInterface::Unregister(p);
}


void aMussel::SetMeshFilename (std::string filename)
{
    meshFilename = filename;
}

void aMussel::Register (RenderOSG* r)
{
    RenderOSGInterface::Register (r);

    osg::ref_ptr<osg::Geode> geode; 
    osg::ref_ptr<osg::Cylinder> cylinder; 
    osg::ref_ptr<osg::ShapeDrawable> cylinderDrawable;

    cylinder = new osg::Cylinder(osg::Vec3(0,0,0),0.1,0.75); 
    cylinderDrawable = new osg::ShapeDrawable(cylinder ); 
    geode = new osg::Geode; 
    geode->addDrawable(cylinderDrawable); 
    aMusselNode = geode;
    
    // one transform for spatial position, will be adjusted to data from physics engine
    transform = new osg::MatrixTransform();
    
    // one transform for scaling model
    osg::ref_ptr<osg::MatrixTransform> transformScale = new osg::MatrixTransform();
    osg::Matrix m = osg::Matrix::identity();
    float coeff = 0.66;
    m.makeScale(osg::Vec3(physicsBullet->scalingFactor * coeff, physicsBullet->scalingFactor * coeff, physicsBullet->scalingFactor * coeff));
    transformScale->setMatrix(m);

    transform->addChild (transformScale);
    transformScale->addChild (aMusselNode);
    r->root->addChild(transform);
}

void aMussel::Unregister (RenderOSG* r)
{
    RenderOSGInterface::Unregister(r);
}

void aMussel::Register (EnergyManager* m)
{
    EnergyManagerInterface::Register(m);
}

void aMussel::Unregister (EnergyManager* m)
{
    EnergyManagerInterface::Unregister(m);
}

void aMussel::Register (WaterVolume* w)
{
    WaterVolumeInterface::Register(w);

    // update quadratic drag with new fluid density
    setDragQuadraticCoefficients(m_linearQuadraticDrag, m_angularQuadraticDrag, w->density);
}

void aMussel::Unregister (WaterVolume* w)
{
    WaterVolumeInterface::Unregister(w);
}

void aMussel::SetPosition (btVector3 p)
{
    btTransform t = body->getCenterOfMassTransform();
    t.setOrigin(p);
    body->setCenterOfMassTransform(t);

    body->setLinearVelocity(btVector3(0,0,0));
    body->setAngularVelocity(btVector3(0,0,0));
}

btVector3 aMussel::GetPosition ()
{
    return body->getCenterOfMassTransform().getOrigin();
}


void aMussel::SetRotation (btQuaternion q)
{
    btTransform t = body->getCenterOfMassTransform();
    t.setRotation(q);
    body->setCenterOfMassTransform(t);

    body->setLinearVelocity(btVector3(0,0,0));
    body->setAngularVelocity(btVector3(0,0,0));
}

btQuaternion aMussel::GetRotation (btQuaternion q)
{
    return body->getOrientation();
}

void aMussel::SetColor (float r, float g, float b, float a)
{
    this->colr = r;
    this->colg = g;
    this->colb = b;
    this->cola = a;
}

void aMussel::Step ()
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
void aMussel::setAddedMass (const btVector3& linear, const btVector3& angular)
{
    m_linearAddedMass = linear;
    m_angularAddedMass = angular;
    updateInertiaTensor ();
}

// // compund shape is the other way, but I want to make faster calculations
// void aMussel::setCenterOfVolume(const btVector3& position) 
// {
//     m_centerOfVolume = position;
//     updateInertiaTensor ();
// }

// drag coefficients must be set as 
// linear part : area exposed * coefficient
// angular part : coefficient only
void aMussel::setDragCoefficients (const btVector3& linear, const btVector3& angular)
{
    m_linearDrag = -linear;
    m_angularDrag = -angular;
}

void aMussel::setDragQuadraticCoefficients (const btVector3& qlinear, const btVector3& qangular, float fluidDensity)
{
    m_linearQuadraticDrag = -qlinear;
    m_angularQuadraticDrag = -qangular;
}

// void aMussel::setBuoyancyFactor (float f)
// {
//     ballast->SetBuoyancyFactor(f);
//     updateBuoyancyForce (ballast->volume);
// }

// void aMussel::updateBuoyancyForce (btScalar volume)
// {
//     m_buoyancy = body->getGravity() * volume * m_fluidDensity * -1.0;
//     // TODO
// //    m_buoyancy = btVector3(0,0,0);
//     m_volume = volume;
// }


void aMussel::updateInertiaTensor ()
{
    body->setMassProps (mass, m_inertia);
    body->updateInertiaTensor();
}
