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

#ifndef DEVICE_PROPELLERS_H
#define DEVICE_PROPELLERS_H

#include "Device.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"

#include "RenderOpenGLInterface.h"

class DevicePropellers : public Device, public RenderOpenGLInterface
{
public : 
    btRigidBody* body;    

    float leftSpeed;
    float rightSpeed;

    btVector3 leftPosition;
    btVector3 rightPosition;

    float maxForce;

    DevicePropellers(PhysicsBullet* p, btRigidBody* body, btVector3 leftPosition, btVector3 rightPosition, float maxForce);
    ~DevicePropellers ();

    void ActionStep();
    void PerceptionStep();
    void Reset();
    void Draw (RenderOpenGL* r);

    float GetLeftSpeed ();
    float GetRightSpeed ();
    void SetSpeed (float left, float right);
    void SetLeftSpeed (float s);
    void SetRightSpeed (float s);
};



#endif
