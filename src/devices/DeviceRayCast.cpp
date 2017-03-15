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

#include "DeviceRayCast.h"


DeviceRayCast::DeviceRayCast(PhysicsBullet* p, btRigidBody* b, btVector3 position, btVector3 direction, btScalar range, int collisionFilter, int collisionType) : 
    RayResultCallback ()    
{
    this->physics = p;
    this->parentBody = b;
    this->position = position;
    this->direction = direction;
    this->range = range;

    // local ray coordinates
    rayFromLocal = position;
    rayToLocal = position + direction * range;

    // inherited from ContactResultCallback
    m_collisionFilterMask = collisionFilter;
    m_collisionFilterGroup = collisionType;

    // lazy, on demand calculation
    calculated = false;
}   
 


DeviceRayCast::~DeviceRayCast()
{

}

void DeviceRayCast::actionStep ()
{
}

void DeviceRayCast::perceptionStep ()
{
    calculated = false;

    // lazy on demand... code moved to query
}

void DeviceRayCast::reset()
{
    calculated = false;
}

float DeviceRayCast::getValue ()
{
    if (!calculated)
    {
	// calculate ray from / ray to in world coords
	rayFromWorld = parentBody->getCenterOfMassTransform() * rayFromLocal; 
	rayToWorld = parentBody->getCenterOfMassTransform() * rayToLocal;    
	
	m_collisionObject = 0;
	m_closestHitFraction = 1.0;
	physics->m_dynamicsWorld->rayTest(rayFromWorld, rayToWorld, (*this));
	value = 1.0 - m_closestHitFraction;
	
	calculated = true;
    }
    return value;
}
    

void DeviceRayCast::draw (RenderOpenGL* r)
{   
    btTransform transform;
    btScalar m[16];
    btMatrix3x3 rot;
    rot.setIdentity();

    transform = parentBody->getWorldTransform();
    transform.getOpenGLMatrix(m);

    glPushMatrix();     
    glMultMatrixf(m);

    dsRotationMatrix rotmat;
    float pos[3];	
    pos[0] = 0;
    pos[1] = 0;
    pos[2] = 0; 
    
    dRotation2dsRotationMatrix(0, rotmat);
    
    r->dsSetColorAlpha (0, 0, 1, 0.3);
    r->dsSetTexture (0);

    // draw the ray
    float pos1[3] = {0,0,0};	
    float pos2[3] = {0,0,0};	

    pos1[0] = rayFromLocal.x(); pos1[1] = rayFromLocal.y(); pos1[2] = rayFromLocal.z(); 
    btVector3 colpos = rayFromLocal + direction * m_closestHitFraction * range;
    pos2[0] = colpos.x(); pos2[1] = colpos.y(); pos2[2] = colpos.z(); 

    glLineWidth (0.5);
    r->dsSetColor (0.5, 0.5, 0.5);
    r->dsSetTexture (0);
    r->dsDrawLine (pos1, pos2);

    glPopMatrix();
}


btScalar DeviceRayCast::addSingleResult(btCollisionWorld::LocalRayResult& rayResult,bool normalInWorldSpace)
{    
    //caller already does the filter on the m_closestHitFraction
    // btAssert(rayResult.m_hitFraction <= m_closestHitFraction);    
    m_closestHitFraction = rayResult.m_hitFraction;    
    m_collisionObject = rayResult.m_collisionObject;
    return 1.f;
}

void DeviceRayCast::setRange(float range)
{
    this->range = range;
    rayToLocal = position + direction * range;    
}
