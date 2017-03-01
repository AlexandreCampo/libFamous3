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


#ifndef DEVICE_BALLAST_H
#define DEVICE_BALLAST_H

#include "Device.h"
#include "WaterVolume.h"
#include "PhysicsBullet.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"


class DeviceBallast : public Device
{
public : 
    PhysicsBullet* physics;
    WaterVolume* waterVolume;
    btRigidBody* body;

    btVector3 position;
    float minVolume;
    float maxVolume;
    float neutralVolume;
    float volume;
    float immersedVolume;
    float buoyancyFactor;
    float sphereRadius;
    float actionCoefficient;

    btVector3 buoyancyForce;
    btVector3 buoyancyTorque;

    DeviceBallast(btVector3 position, PhysicsBullet* p, WaterVolume* w, btRigidBody* b, float minVolume, float maxVolume, float actionCoefficient = 1.0);
    ~DeviceBallast ();

    void PerceptionStep();
    void ActionStep();
    void Reset();
    
    void SetBuoyancyFactor (float f);
    btVector3 GetBuoyancyForce ();
};



#endif
