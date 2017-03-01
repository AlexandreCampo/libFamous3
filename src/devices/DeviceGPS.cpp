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

#include "DeviceGPS.h"
#include "Simulator.h"
#include <iostream>

DeviceGPS::DeviceGPS(btVector3 position, PhysicsBullet* p, btRigidBody* b) :
    Device()
{
    // time step will follow controller's
    
    this->position = position;
    this->physics = p;
    this->body = b;
}


DeviceGPS::~DeviceGPS()
{
}

void DeviceGPS::PerceptionStep()
{
    calculated = false;
}

void DeviceGPS::ActionStep()
{
}

void DeviceGPS::Reset()
{
    calculated = false;
}
   
btVector3 DeviceGPS::GetGlobalPosition()
{    
    if (!calculated)
    {
	globalPosition = body->getCenterOfMassTransform() * position;
	
	calculated = true;
    }

    return globalPosition;
}
