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

#ifndef DEVICE_DOCKER_H
#define DEVICE_DOCKER_H


#include "Device.h"

#include "PhysicsBullet.h"
#include "RenderOSGInterface.h"

#include <list>

class DeviceDocker : public Device, public btBroadphaseAabbCallback, public RenderOSGInterface
{
public :
    PhysicsBullet* physics;
    btRigidBody* parentBody;
    btTransform localTransform;
    int detectionCollisionFilter;
    int detectionCollisionType;
    int dockingCollisionFilter;
    int dockingCollisionType;

    btRigidBody* collisionBody; 
    btCollisionShape* collisionShape;

    float maxDockingRange;
    float maxDetectionRange;
    float verticalTolerance;

    bool dockable = true;
    bool calculated = false;
    
    std::list<DeviceDocker*> detectedDevices;
    std::map<DeviceDocker*, btFixedConstraint*> dockedTo;
    std::map<DeviceDocker*, btFixedConstraint*> dockedBy;


    DeviceDocker(PhysicsBullet* p, btRigidBody* b, btTransform t, float maxDetectionRange, float maxDockingRange, float verticalTolerance, int detectionCollisionFiler, int dockingCollisionFilter, int detectionCollisionType, int dockingCollisionType);
    
    ~DeviceDocker();

    void actionStep ();
    void perceptionStep ();
    void reset();

    std::list<DeviceDocker*>& detectDockableDevices();
    bool dock(DeviceDocker* dockable);
    bool undock(DeviceDocker* dockable);

    virtual bool process (const btBroadphaseProxy *proxy);
    
    void setDockable(bool d);
    bool isDockable();

    void draw (RenderOSG* r);
};


#endif
