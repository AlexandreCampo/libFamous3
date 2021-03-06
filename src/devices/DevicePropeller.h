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

#ifndef DEVICE_PROPELLER_H
#define DEVICE_PROPELLER_H

#include "Device.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"

#include "PhysicsBullet.h"
#include "RenderOSGInterface.h"

// store only one and same geometry for all such devices
static osg::ref_ptr<osg::Node> devicePropellerNode = NULL;

class DevicePropeller : public Device, public RenderOSGInterface
{
public : 
    btRigidBody* body;    

    float speed;
    float force;
    btTransform transform;

    float maxForce;

    bool drawable;
    
    DevicePropeller(PhysicsBullet* p, btRigidBody* body, float maxForce);
    ~DevicePropeller ();

    void actionStep();
    void perceptionStep();
    void reset();
    void draw (RenderOSG* r);
    
    float getSpeed ();
    void setSpeed (float s);

    void setPosition(btVector3 position);
    btVector3 getPosition();
    void setOrientation(btQuaternion q);
    btQuaternion getOrientation();

    void registerService (RenderOSG* r);
    void unregisterService (RenderOSG* r);

    void setDrawable(bool d);
    bool isDrawable();

};



#endif
