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

#ifndef DEVICE_RAY_CAST_H
#define DEVICE_RAY_CAST_H

// WARNING : this is a lazy sensor ... always call get value first, for instance has hit must be called afterwards only


#include "Device.h"
#include "PhysicsBullet.h"
#include "RenderOpenGLInterface.h"

#include <vector>
#include <iostream>

class DeviceRayCast : public Device, public btCollisionWorld::RayResultCallback, public RenderOpenGLInterface
{
public :

    PhysicsBullet* physics;
    btRigidBody* parentBody;
    btVector3 direction;
    btVector3 position;   
    btScalar range;

    bool calculated;
    float value;

    // these 2 are already present in ray result callback
    /* int collisionFilter; */
    /* int collisionType; */

    btRigidBody* collisionBody; 
    // btCollisionShape* collisionShape;

    // to deal with rays
    btVector3   rayFromLocal;
    btVector3   rayToLocal;

    btVector3	rayFromWorld;
    btVector3	rayToWorld;
    
    // btVector3	hitNormalWorld;
    // btVector3	hitPointWorld;


    DeviceRayCast(PhysicsBullet* p, btRigidBody* b, btVector3 position, btVector3 direction, btScalar range, int collisionFilter, int collisionType = 0x7FFFFFFF);
    ~DeviceRayCast();

    void actionStep ();
    void perceptionStep ();
    void reset();
    
    float getValue();

    btScalar addSingleResult(btCollisionWorld::LocalRayResult& rayResult,bool normalInWorldSpace);
   
    void draw (RenderOpenGL* r);

    void setRange(float range);
};


#endif
