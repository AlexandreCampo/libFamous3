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

#include "DevicePressure.h"
#include "Simulator.h"
#include <iostream>

DevicePressure::DevicePressure(btVector3 position, PhysicsBullet* p, WaterVolume* w, btRigidBody* b) :
    Device()
{
    // time step will follow controller's
    
    this->position = position;
    this->physics = p;
    this->waterVolume = w;
    this->body = b;
}


DevicePressure::~DevicePressure()
{
}

void DevicePressure::perceptionStep()
{
    calculated = false;
}

void DevicePressure::actionStep()
{
}

void DevicePressure::reset()
{
    calculated = false;
}
   
float DevicePressure::getValue()
{
    
    if (!calculated)
    {
	btVector3 worldpos = body->getCenterOfMassTransform() * position;
	float height = waterVolume->getHeight(worldpos, object->simulator->time);
	
	pressure = height * body->getGravity().getZ() * waterVolume->density;
	
	calculated = true;
    }

    return pressure;
}

