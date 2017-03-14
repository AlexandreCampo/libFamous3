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

#ifndef PHYSICS_BULLET_INTERFACE_H
#define PHYSICS_BULLET_INTERFACE_H

#include "PhysicsBullet.h"
#include "Object.h"

#include "BulletDynamics/Dynamics/btDynamicsWorld.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletCollision/CollisionShapes/btCylinderShape.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "LinearMath/btDefaultMotionState.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btVector3.h"


class PhysicsBulletInterface
{
public:

    PhysicsBullet* physicsBullet = NULL;
    
    int collisionType;
    int collisionFilter;

    PhysicsBulletInterface();
    
    virtual void Register (PhysicsBullet* p) = 0;
    virtual void Unregister (PhysicsBullet* p) = 0;    

    int GetCollisionType ();
    void SetCollisionType (int type);

    int GetCollisionFilter ();
    void SetCollisionFilter (int filter);

//    virtual void CustomPhysics ();
};


#endif
