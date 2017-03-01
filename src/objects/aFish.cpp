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

#include "aFish.h"
#include "Simulator.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"

#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/Material>

#include <iostream>

aFish::aFish() :
    Object(),
    PhysicsBulletInterface(),
    WaterVolumeInterface(),
    RenderOpenGLInterface(),
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

    displayList = 0;
}
 


aFish::~aFish()
{
}

void aFish::AddDevices()
{
    propellerLeft = new DevicePropeller (physicsBullet, body, 1);
    propellerLeft->SetPosition(m_centerOfVolume + btVector3( 0, -dimensions[1] / 2.0, 0 ));
    propellerLeft->SetOrientation(btQuaternion( 0, 0, 0 ));
    if (renderOSG) propellerLeft->Register(renderOSG);
    propellerLeft->SetDrawable(false);
    Add(propellerLeft);
    
    propellerRight = new DevicePropeller (physicsBullet, body, 1);
    propellerRight->SetPosition(m_centerOfVolume + btVector3( 0, dimensions[1] / 2.0, 0 ));
    propellerRight->SetOrientation(btQuaternion( 0, 0, 0 ));
    if (renderOSG) propellerRight->Register(renderOSG);
    propellerRight->SetDrawable(false);
    Add(propellerRight);
    
    // propellers = new DevicePropellers(physicsBullet, body, leftPos, rightPos, 0.1 * physicsBullet->scalingFactor);
    // Add (propellers);

    float neutralVolume = (1.0 / body->getInvMass()) / waterVolume->density;
    float ballastVolume = (0.03 * 0.03 * 0.2 * physicsBullet->scalingFactor * physicsBullet->scalingFactor * physicsBullet->scalingFactor);

    ballast = new DeviceBallast (m_centerOfVolume, physicsBullet, waterVolume, body, neutralVolume - ballastVolume / 2.0, neutralVolume + ballastVolume / 2.0);
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

    // front left
    btVector3 pos (0.0, 0.0, 0.0);
    btVector3 dir (1, -sin(a), 0);   
    optical->AddTransmitter (pos, dir, range, 30.0 * M_PI / 180.0);

    // front right
    pos = btVector3 (0.0, 0.0, 0.0);
    dir = btVector3 (1, sin(a), 0);   
    optical->AddTransmitter (pos, dir, range, 30.0 * M_PI / 180.0);

    // left
    pos = btVector3 (0.0, 0.0, 0.0);
    dir = btVector3 (0, -1, 0);   
    optical->AddTransmitter (pos, dir, range, 30.0 * M_PI / 180.0);

    // right
    pos = btVector3 (0.0, 0.0, 0.0);
    dir = btVector3 (0, 1, 0);   
    optical->AddTransmitter (pos, dir, range, 30.0 * M_PI / 180.0);

    // front
    pos = btVector3 (0.0, 0.0, 0.0);
    dir = btVector3 (1, 0, 0);   
    optical->AddTransmitter (pos, dir, range, 30.0 * M_PI / 180.0);

    // back
    pos = btVector3 (0.0, 0.0, 0.0);
    dir = btVector3 (-1, 0, 0);   
    optical->AddTransmitter (pos, dir, range, 30.0 * M_PI / 180.0);

    // top
    pos = btVector3 (0.0, 0.0, 0.0);
    dir = btVector3 (0, 0, 1);   
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
    
    pos = btVector3 (0.1 * physicsBullet->scalingFactor - addedrange, -0.005  * physicsBullet->scalingFactor, 0.005  * physicsBullet->scalingFactor);
    dir = btVector3 (px, -pyz, pyz);   
    rayFrontLU = new DeviceRayCast (physicsBullet, body, pos, dir, range, cf3, ct3);
    Add (rayFrontLU);
    
    pos = btVector3 (0.1 * physicsBullet->scalingFactor - addedrange, -0.005  * physicsBullet->scalingFactor, -0.005  * physicsBullet->scalingFactor);
    dir = btVector3 (px, -pyz, -pyz);   
    rayFrontLD = new DeviceRayCast (physicsBullet, body, pos, dir, range, cf3, ct3);
    Add (rayFrontLD);

    pos = btVector3 (0.1 * physicsBullet->scalingFactor - addedrange, 0.005  * physicsBullet->scalingFactor, 0.005  * physicsBullet->scalingFactor);
    dir = btVector3 (px, pyz, pyz);   
    rayFrontRU = new DeviceRayCast (physicsBullet, body, pos, dir, range, cf3, ct3);
    Add (rayFrontRU);

    pos = btVector3 (0.1 * physicsBullet->scalingFactor - addedrange, 0.005  * physicsBullet->scalingFactor, -0.005  * physicsBullet->scalingFactor);
    dir = btVector3 (px, pyz, -pyz);   
    rayFrontRD = new DeviceRayCast (physicsBullet, body, pos, dir, range, cf3, ct3);
    Add (rayFrontRD);
    
    pos = btVector3 (0.0, -0.025 * physicsBullet->scalingFactor + addedrange, 0.0);
    dir = btVector3 (0.0, -1.0, 0.0);   
    rayLeft = new DeviceRayCast (physicsBullet, body, pos, dir, range, cf3, ct3);  
    Add (rayLeft);
    
    pos = btVector3 (0.0, 0.025 * physicsBullet->scalingFactor - addedrange, 0.0);
    dir = btVector3 (0.0, 1.0, 0.0);   
    rayRight = new DeviceRayCast (physicsBullet, body, pos, dir, range, cf3, ct3);  
    Add (rayRight);
    
    pos = btVector3 (0.0, 0.0, 0.05 * physicsBullet->scalingFactor - addedrange);
    dir = btVector3 (0.0, 0.0, 1.0);   
    rayTop = new DeviceRayCast (physicsBullet, body, pos, dir, range, cf3, ct3);  
    Add (rayTop);
    
    pos = btVector3 (0.0, 0.0, -0.05 * physicsBullet->scalingFactor + addedrange);
    dir = btVector3 (0.0, 0.0, -1.0);   
    rayBottom = new DeviceRayCast (physicsBullet, body, pos, dir, range, cf3, ct3);  
    Add (rayBottom);
    
    pos = btVector3 (-0.1 + addedrange, 0.0, 0);
    dir = btVector3 (-1.0, 0.0, 0.0);   
    rayBack = new DeviceRayCast (physicsBullet, body, pos, dir, range, cf3, ct3);  
    Add (rayBack);


    networker = new DeviceNetworker ();    
    Add (networker);
    
}

void aFish::Draw (RenderOSG* r)
{
    btScalar ogl[16];
    btTransform t = body->getCenterOfMassTransform() * principalTransform.inverse();
    t.getOpenGLMatrix( ogl );
    osg::Matrix m(ogl);
    RenderOSGInterface::transform->setMatrix (m);

    osg::ref_ptr<osg::Material> mat = new osg::Material;
    mat->setDiffuse (osg::Material::FRONT_AND_BACK, osg::Vec4(colr, colg, colb, cola));
    aFishNode->getOrCreateStateSet()->setAttributeAndModes(mat, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
}

void aFish::Draw (RenderOpenGL* r)
{
    btScalar m[16];
    btMatrix3x3 rot;
    rot.setIdentity();
    
    btTransform t = body->getCenterOfMassTransform() * principalTransform.inverse();
    t.getOpenGLMatrix(m);
        
   glPushMatrix(); 
   glMultMatrixf(m);
    
   r->setColor (colr, colg, colb, 1);
   //glColor3f(1,0,0); 
   
   if (displayList > 0)
   {
       glCallList(displayList);    
//       glutSolidCube (dimensions[0]/8.0);
   }
   else
   {
       glutSolidCube (dimensions[0]/4.0);
//    r->dsSetColor (1, 0, 0);
//    r->dsSetTexture (0);
    // pos[0] = dimensions[0] * 4.0 / 5.0 / 2.0;
    // dims[0] *= 1.0 / 5.0;
    // glShadeModel (GL_FLAT);
    // r->setTransform (pos,rotmat);
    // r->drawBox (dims);
    // glPopMatrix();
//    r->dsDrawBox((float*) &pos, (float*) &rotmat, (float*) &dims);
       
   }

   glPopMatrix();
    
   // draw devices ...
   // acoustic->Draw(r);
   // optical->Draw(r);
   // proximitySensor->Draw(r);
   // propellers->Draw(r);
   // rayFrontLU->Draw(r);
   // rayFrontLD->Draw(r);
   // rayFrontRU->Draw(r);
   // rayFrontRD->Draw(r);
   // rayBack->Draw(r);
   // rayLeft->Draw(r);
   // rayRight->Draw(r);
   // rayTop->Draw(r);
   // rayBottom->Draw(r);
}



void aFish::Register (PhysicsBullet* p)
{
    PhysicsBulletInterface::Register(p);
    
    SetTimeStep(p->GetTimeStep());
//    p->SetCustomPhysics(true);

    dimensions[0] = 0.35 * physicsBullet->scalingFactor;
    dimensions[1] = 0.10 * physicsBullet->scalingFactor;
    dimensions[2] = 0.15 * physicsBullet->scalingFactor;

    mass = 1.0;
    btScalar volume = dimensions[0] * dimensions[1] * dimensions[2];
    btScalar linearDamping = 0;
    btScalar angularDamping = 0;

    // we create a first, detailed compound shape in order to calculate the correct inertia tensor
    btCompoundShape* cshape = new btCompoundShape(false);

    float dim1 = dimensions[2] * 0.8;
    float dim2 = dimensions[2] - dim1;
    btBoxShape* boxShapeA = new btBoxShape(btVector3(dimensions[0], dimensions[1], dim1) / 2.0);
    btTransform localTransform;
    localTransform.setIdentity();
    localTransform.setOrigin(btVector3(0,0, dim1 / 2.0 + dim2 - dimensions[2] / 2.0));
    cshape->addChildShape(localTransform,boxShapeA);

    btBoxShape* boxShapeB = new btBoxShape(btVector3(dimensions[0], dimensions[1], dim2) / 2.0);
    localTransform.setIdentity();
    localTransform.setOrigin (btVector3(0, 0, dim2 / 2.0 - dimensions[2] / 2.0));
    cshape->addChildShape(localTransform,boxShapeB);

    btScalar* masses = new btScalar [2];
    masses[0] = 0.2 * mass;
    masses[1] = 0.8 * mass;

    cshape->calculatePrincipalAxisTransform(masses, principalTransform, m_inertia);
    m_centerOfVolume = principalTransform.inverse().getOrigin();
    
    // we can delete previous shapes
    delete cshape;
    delete boxShapeA;
    delete boxShapeB;
    delete [] masses;

    // now the real shape used used, a simplified compound shape that will be used for collisions
    // create a new compound with world transform/center of mass properly aligned with the principal axis   
    cshape = new btCompoundShape(false); 
    btBoxShape* boxShape = new btBoxShape(btVector3(dimensions[0], dimensions[1], dimensions[2]) / 2.0);
    
    localTransform.setIdentity();
    btTransform newLocalTransform = principalTransform.inverse() * localTransform;
    cshape->addChildShape(newLocalTransform,boxShape);
       
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
    body->setActivationState(DISABLE_DEACTIVATION);

    setDragCoefficients(btVector3(0,0,0), btVector3(0,0,0));
    setDragQuadraticCoefficients(btVector3(0,0,0), btVector3(0,0,0), 1.0);
    setAddedMass (btVector3 (0,0,0), btVector3 (0,0,0));

    p->m_dynamicsWorld->addRigidBody(body, collisionType, collisionFilter);
}

void aFish::Unregister (PhysicsBullet* p)
{
}

void aFish::Register (RenderOpenGL* r)
{
    RenderOpenGLInterface::Register(r);
}

void aFish::Unregister (RenderOpenGL* r)
{
    RenderOpenGLInterface::Unregister(r);
}

void aFish::SetMeshFilename (std::string filename)
{
    meshFilename = filename;
}

void aFish::Register (RenderOSG* r)
{
    RenderOSGInterface::Register (r);

    if (meshFilename.empty())
    {
	cout << "No 3d model defined for OSG Render..." << endl;
	return;
    }

    if (!aFishNode) aFishNode = osgDB::readNodeFile(meshFilename);
//    if (!aFishNode) aFishNode = osgDB::readNodeFile(3dmodelFilename + "aFish.3ds.(0.01,0.01,0.01).scale");
    if (!aFishNode)
    {
        std::cout << "Could not load aFish model." << std::endl;
    }    

    // one transform for spatial position, will be adjuted to data from physics engine
    RenderOSGInterface::transform = new osg::MatrixTransform();
    
    // one transform for scaling model
    osg::ref_ptr<osg::PositionAttitudeTransform> t = new osg::PositionAttitudeTransform();
    t->setScale(osg::Vec3(1.2, 1.2, 1.2));

    RenderOSGInterface::transform->addChild (t);
    t->addChild (aFishNode);
    r->root->addChild(RenderOSGInterface::transform);
}

void aFish::Unregister (RenderOSG* r)
{
    RenderOSGInterface::Unregister(r);
}

void aFish::Register (EnergyManager* m)
{
    EnergyManagerInterface::Register(m);
}

void aFish::Unregister (EnergyManager* m)
{
    EnergyManagerInterface::Unregister(m);
}

void aFish::Register (WaterVolume* w)
{
    WaterVolumeInterface::Register(w);
    
    // update quadratic drag with new fluid density
    setDragQuadraticCoefficients(m_linearQuadraticDrag, m_angularQuadraticDrag, w->density);
}

void aFish::Unregister (WaterVolume* w)
{
    WaterVolumeInterface::Unregister(w);
}



void aFish::SetPosition (btVector3 p)
{
    btTransform t = body->getCenterOfMassTransform();
    t.setOrigin(p);
    body->setCenterOfMassTransform(t);

    body->setLinearVelocity(btVector3(0,0,0));
    body->setAngularVelocity(btVector3(0,0,0));
}

btVector3 aFish::GetPosition ()
{
    return body->getCenterOfMassTransform().getOrigin();
}


void aFish::SetRotation (btQuaternion q)
{
    btTransform t = body->getCenterOfMassTransform();
    t.setRotation(q);
    body->setCenterOfMassTransform(t);

    body->setLinearVelocity(btVector3(0,0,0));
    body->setAngularVelocity(btVector3(0,0,0));
}

btQuaternion aFish::GetRotation (btQuaternion q)
{
    return body->getOrientation();
}


void aFish::SetColor (float r, float g, float b, float a)
{
    this->colr = r;
    this->colg = g;
    this->colb = b;
    this->cola = a;
}

void aFish::Step ()
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
void aFish::setAddedMass (const btVector3& linear, const btVector3& angular)
{
    m_linearAddedMass = linear;
    m_angularAddedMass = angular;
    updateInertiaTensor ();
}

// drag coefficients must be set as 
// linear part : area exposed * coefficient
// angular part : coefficient only
void aFish::setDragCoefficients (const btVector3& linear, const btVector3& angular)
{
    m_linearDrag = -linear;
    m_angularDrag = -angular;
}

void aFish::setDragQuadraticCoefficients (const btVector3& qlinear, const btVector3& qangular, float fluidDensity)
{
    m_linearQuadraticDrag = -qlinear;
    m_angularQuadraticDrag = -qangular;
}


void aFish::updateInertiaTensor ()
{
    body->setMassProps (mass, m_inertia);
    body->updateInertiaTensor();
}
