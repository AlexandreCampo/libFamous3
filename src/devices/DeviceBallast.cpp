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

#include "DeviceBallast.h"
#include "Simulator.h"
#include <iostream>

DeviceBallast::DeviceBallast(btVector3 position, PhysicsBullet* p, WaterVolume* w, btRigidBody* b, float minVolume, float maxVolume, float actionCoefficient) :
    Device()
{
    this->position = position;
    this->physics = p;
    this->waterVolume = w;
    this->body = b;
    setActionTimestep(p->getTimestep());

    this->minVolume = minVolume;
    this->maxVolume = maxVolume;
    this->actionCoefficient = actionCoefficient;
    
    // calculate volume of fluid to make up for the mass of the body
    neutralVolume = (maxVolume + minVolume) / 2.0;

    setBuoyancyFactor(0.0);
}


DeviceBallast::~DeviceBallast()
{
}

// the factor is in [-1; 1] , 0 is neutral buoyancy
void DeviceBallast::setBuoyancyFactor (float f)
{
    buoyancyFactor = f;
    
    volume = minVolume + (maxVolume - minVolume) * (f + 1.0) / 2.0;

    // approximate body shape with a sphere
    sphereRadius = cbrt(volume * 3.0 / 4.0 / M_PI);
}

void DeviceBallast::perceptionStep()
{
}

void DeviceBallast::actionStep()
{
    btVector3 worldpos = body->getCenterOfMassTransform() * position;        
    
    // request water height at ballast position
    float height = waterVolume->getHeight(worldpos, object->simulator->time);
    
    // calculate how much of the body is immersed
    float dmax = worldpos.getZ() + sphereRadius * actionCoefficient - height;
    if (dmax > 0)
    {
	// fully emerged
	if (dmax >= sphereRadius * actionCoefficient * 2)
	{
	    immersedVolume = 0;
	}
	else
	{
	    // Volume = (pi/3)H^2(3R - H)
	    if (dmax > sphereRadius * actionCoefficient)
	    {
	    	float h = sphereRadius * 2.0 - dmax / actionCoefficient;
	    	immersedVolume = M_PI * h * h * (3.0 * sphereRadius - h) / 3.0;
	    }
	    else
	    {
	    	float h = dmax / actionCoefficient;
	    	float vcap = M_PI * h * h * (3.0 * sphereRadius - h) / 3.0;
	    	immersedVolume = volume - vcap;
	    }
	}	
	
    }
    // fully immersed
    else
    {
	immersedVolume = volume;
    }

    buoyancyForce = -body->getGravity() * immersedVolume * waterVolume->density;

    const btMatrix3x3& transform = body->getCenterOfMassTransform().getBasis();
    btVector3 relpos = transform * position;
    buoyancyTorque = relpos.cross(buoyancyForce * body->getLinearFactor());

    body->applyCentralForce(buoyancyForce);
    body->applyTorque(buoyancyTorque);
}

void DeviceBallast::reset()
{
    setBuoyancyFactor (0.0);    
}
   

