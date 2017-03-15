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

#ifndef DEVICE_ACOUSTIC_TRANSCEIVER_H
#define DEVICE_ACOUSTIC_TRANSCEIVER_H


#include "Device.h"

#include "PhysicsBullet.h"
#include "RenderOpenGLInterface.h"

#include <list>

class DeviceAcousticTransceiver : public Device, public btBroadphaseAabbCallback, public RenderOpenGLInterface
{
public :
    PhysicsBullet* physics;
    btRigidBody* parentBody;
    btTransform localTransform;
    int collisionFilter;
    int collisionType;

    btRigidBody* collisionBody; 
    btCollisionShape* collisionShape;

    float maxRange;

    struct Message
    {
	Message (float d, int c) 
	    {
		// time = t;
		distance = d;
		/* localPosition = lp; */
		content = c;
	    }
	
	/* float time; */
	float distance;
//	btVector3 localPosition;
	int content;
    };

    std::list<Message> messagesReceived;
    std::list<Message> messagesSent;


    DeviceAcousticTransceiver(PhysicsBullet* p, btRigidBody* b, btTransform t, int collisionFilter, int collisionType = 2^30, float maxRange = 1.0);
    ~DeviceAcousticTransceiver();

    void send (int content);
    bool receive (int& c);

    void actionStep ();
    void perceptionStep ();
    void reset();

    virtual bool process (const btBroadphaseProxy *proxy);

    void draw (RenderOpenGL* r);
};


#endif
