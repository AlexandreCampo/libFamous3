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
#include "RenderOSGInterface.h"

#include <list>
#include <vector>

// store only one and same geometry for all such devices
static osg::ref_ptr<osg::Node> deviceOpticalTransceiverNode = NULL;

class DeviceOpticalTransceiver : public Device, public btBroadphaseAabbCallback, public RenderOSGInterface
{
public :
    PhysicsBullet* physics;
    btRigidBody* body;

    btTransform transform;
    btRigidBody* collisionBody; 
    btCollisionShape* collisionShape;

    int collisionFilter;
    int collisionType;

    float maxRange;
    float value;
    bool receiveOmnidirectional;

    bool drawable;

    struct Transmitter
    {
	float apertureAngle;
	float cosApertureAngle;
	btVector3 position;
	btVector3 direction;
	
	Transmitter (btVector3 position, btVector3 direction, btScalar range, btScalar angle)
	    {
		this->position = position;
		this->direction = direction.normalize();

		apertureAngle = angle;
		cosApertureAngle = cos (apertureAngle);
	    }
    };
    std::vector<Transmitter> transmitters;

    struct Receiver
    {
	float apertureAngle;
	float cosApertureAngle;
	btVector3 position;
	btVector3 direction;

	Receiver (btVector3 position, btVector3 direction, btScalar range, btScalar angle)
	    {
		this->position = position;
		this->direction = direction.normalize();
		apertureAngle = angle;
		cosApertureAngle = cos (apertureAngle);
	    }
    };
    std::vector<Receiver> receivers;


    struct Message
    {
	Message (float d, int c, int tr) 
	    {
		distance = d;
		content = c;
		transmitter = tr;
		receiver = -1;
	    }
	
	float distance;
	int content;
	int transmitter;
	int receiver;
    };

    std::list<Message> messagesReceived;
    std::list<Message> messagesSent;


    DeviceOpticalTransceiver(PhysicsBullet* p, btRigidBody* b, btTransform t, int collisionFilter, int collisionType = 2^30, float maxRange = 1.0); 
    ~DeviceOpticalTransceiver();

    void AddTransmitter (btVector3 position, btVector3 direction, btScalar range, btScalar angle);
    void AddReceiver (btVector3 position, btVector3 direction, btScalar range, btScalar angle);
    void SetReceiveOmnidirectional(bool omni);
    
    void Send (int content, int transmitter = -1);
    bool Receive (int& c, int& r);

    void ActionStep ();
    bool process (const btBroadphaseProxy *proxy);
    bool processOmniToOmni(Message& m, btCollisionObject* dobj, DeviceOpticalTransceiver* od);
    bool processTransmitterToOmni(Message& m, const Transmitter& tra, btCollisionObject* dobj, DeviceOpticalTransceiver* od);
    bool processOmniToReceivers(Message& m, btCollisionObject* dobj, DeviceOpticalTransceiver* od);
    bool processTransmitterToReceivers(Message& m, const Transmitter& tra, btCollisionObject* dobj, DeviceOpticalTransceiver* od);

    void PerceptionStep ();
    void Reset();

    void SetRange(float range);

    void SetDrawable(bool d);
    bool IsDrawable();
    void Register (RenderOSG* r);
    void Draw (RenderOSG* r);

};


#endif
