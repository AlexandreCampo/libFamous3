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

#ifndef DEVICE_OPTICAL_TRANSCEIVER_H
#define DEVICE_OPTICAL_TRANSCEIVER_H


#include "Device.h"

#include "PhysicsBullet.h"
#include "RenderOpenGLInterface.h"

#include <list>
#include <vector>

#define DEVICE_OPTICAL_TRANSCEIVER_MAX_TRANSMITTERS 20

class DeviceOpticalTransceiver : public Device, public btBroadphaseAabbCallback, public RenderOpenGLInterface
{
public :
    PhysicsBullet* physics;
    btRigidBody* parentBody;

    btTransform localTransform;
    btRigidBody* collisionBody; 
    btCollisionShape* collisionShape;

    int collisionFilter;
    int collisionType;

    float maxRange;
    float value;

    struct Transmitter
    {
	btVector3 rayFromLocal;
	btVector3 rayToLocal;
	float apertureAngle;
	float cosApertureAngle;
	btVector3 direction;

	Transmitter (btVector3 position, btVector3 direction, btScalar range, btScalar angle)
	    {
		rayFromLocal = position;
		rayToLocal = position + direction * range;
		this->direction = direction;
		apertureAngle = angle;
		cosApertureAngle = cos (apertureAngle);
	    }
    };
    std::vector<Transmitter> transmitters;

    struct Message
    {
	Message (float d, int c, float t, int tr) 
	    {
		time = t;
		distance = d;
		content = c;
		transmitter = tr;		
	    }
	
	float time;
	float distance;
	int content;
	int transmitter;
	Object* emitter;
    };

    std::list<Message> messagesReceived;
    std::list<Message> messagesSent;


    DeviceOpticalTransceiver(PhysicsBullet* p, btRigidBody* b, btTransform t, int collisionFilter, int collisionType = 2^30, float maxRange = 1.0); 
    ~DeviceOpticalTransceiver();

    void AddTransmitter (btVector3 position, btVector3 direction, btScalar range, btScalar angle);

    void Send (int content, float time, int transmitter);
    void SendOmnidirectional (int content, float time);
    bool Receive (int& c);

    void ActionStep ();
    bool process (const btBroadphaseProxy *proxy);
    void PerceptionStep ();
    void Reset();

    void Draw (RenderOpenGL* r);

    void SetRange(float range);
};


#endif
