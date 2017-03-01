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

#include "DevicePropellers.h"
#include "RenderOpenGLInterface.h"

#include <GL/glut.h> 
#include <GL/gl.h>   
#include <GL/glu.h>  

#include <iostream>



DevicePropellers::DevicePropellers(PhysicsBullet* p, btRigidBody* body, btVector3 leftPosition, btVector3 rightPosition, float maxForce) : Device()
{
    SetActionTimeStep(p->GetTimeStep());
    
    this->body = body;    
    this->maxForce = maxForce;

    this->leftPosition = leftPosition;
    this->rightPosition = rightPosition;

    leftSpeed = 0.0;
    rightSpeed = 0.0;
}


DevicePropellers::~DevicePropellers()
{

}

void DevicePropellers::SetSpeed(float left, float right)
{
    leftSpeed = left;
    rightSpeed = right;
}

float DevicePropellers::GetLeftSpeed()
{
    return (leftSpeed);
}

float DevicePropellers::GetRightSpeed()
{
    return (rightSpeed);
}


void DevicePropellers::SetLeftSpeed(float s)
{
    leftSpeed = s;
}

void DevicePropellers::SetRightSpeed(float s)
{
    rightSpeed = s;
}

void DevicePropellers::ActionStep()
{
    btTransform t = body->getCenterOfMassTransform();
    btVector3 f = t.getBasis().getColumn(0) * maxForce;
    btVector3 flocal = btVector3(1,0,0) * maxForce;

    body->activate();
    body->applyCentralForce(f * leftSpeed);
    body->applyCentralForce(f * rightSpeed);
    body->applyTorque(leftPosition.cross(flocal * leftSpeed));
    body->applyTorque(rightPosition.cross(flocal * rightSpeed));
}

void DevicePropellers::PerceptionStep()
{
}

void DevicePropellers::Reset()
{
    leftSpeed = 0;
    rightSpeed = 0;
}

void DevicePropellers::Draw (RenderOpenGL* r)
{   
    btTransform transform;
    btScalar m[16];
    btMatrix3x3 rot;
    rot.setIdentity();

    transform = body->getWorldTransform();
    		
    transform.getOpenGLMatrix(m);

    glPushMatrix();     
    glMultMatrixf(m);

    dsRotationMatrix rotmat;
    float pos[3];	
    pos[0] = 0;
    pos[1] = 0;
    pos[2] = 0; //- width / 2.0;
    
    dRotation2dsRotationMatrix(0, rotmat);
    
    r->dsSetColorAlpha (0, 0, 1, 0.3);
    r->dsSetTexture (0);

    float pos1[3] = {0,0,0};	
    float pos2[3] = {0,0,0};	

    btTransform t = body->getWorldTransform();
    btVector3 f = btVector3(1,0,0) * maxForce * 1.0;
    btVector3 torque = leftPosition.cross(f * leftSpeed) * 1.0;

    pos1[0] = leftPosition.x(); pos1[1] = leftPosition.y(); pos1[2] = leftPosition.z(); 

    pos2[0] = leftPosition.x() + f.x() * leftSpeed; 
    pos2[1] = leftPosition.y() + f.y() * leftSpeed; 
    pos2[2] = leftPosition.z() + f.z() * leftSpeed; 

    r->dsSetColor (1, 0.7, 0);
    r->dsSetTexture (0);
    r->dsDrawLine (pos1, pos2);


    // torque = rightPosition.cross(f * rightSpeed) * 10.0;

    pos1[0] = rightPosition.x(); pos1[1] = rightPosition.y(); pos1[2] = rightPosition.z(); 

    pos2[0] = rightPosition.x() + f.x() * rightSpeed; 
    pos2[1] = rightPosition.y() + f.y() * rightSpeed; 
    pos2[2] = rightPosition.z() + f.z() * rightSpeed; 

    r->dsSetColor (1, 0.7, 0);
    r->dsSetTexture (0);
    r->dsDrawLine (pos1, pos2);


    glPopMatrix();
}
