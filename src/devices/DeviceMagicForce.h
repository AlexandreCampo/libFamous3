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

#ifndef DEVICE_MAGIC_FORCE_H
#define DEVICE_MAGIC_FORCE_H


#include "Device.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"

#include "PhysicsBullet.h"

class DeviceMagicForce : public Device
{
public : 
    btRigidBody* body;    

    btVector3 centralForce;
    
    DeviceMagicForce (PhysicsBullet* p, btRigidBody* body);
    ~DeviceMagicForce ();

    void perceptionStep();
    void actionStep();
    void reset();
    void applyCentralForce (btVector3 force);
};



#endif
