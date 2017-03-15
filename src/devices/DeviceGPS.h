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


#ifndef DEVICE_GPS_H
#define DEVICE_GPS_H

#include "Device.h"
#include "PhysicsBullet.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"


class DeviceGPS : public Device
{
public : 
    PhysicsBullet* physics;
    btRigidBody* body;

    btVector3 position;
    btVector3 globalPosition;
    bool calculated;
    
    DeviceGPS(btVector3 position, PhysicsBullet* p, btRigidBody* b);
    ~DeviceGPS ();

    void perceptionStep();
    void actionStep();
    void reset();

    btVector3 getGlobalPosition();
};



#endif
