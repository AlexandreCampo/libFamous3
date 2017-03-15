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

#ifndef DEVICE_NETWORKER_H
#define DEVICE_NETWORKER_H


#include "Device.h"

#include "PhysicsBullet.h"
#include "RenderOpenGLInterface.h"

#include <list>
#include <vector>


class DeviceNetworker : public Device, public RenderOpenGLInterface
{
public :
    
    std::list<DeviceNetworker*> connections;
    
    struct Message
    {
	Message (int c, float t, DeviceNetworker* s = NULL, DeviceNetworker* d = NULL) 
	    {
		time = t;
		content = c;
		src = s;
		dst = d;
	    }

	DeviceNetworker* dst;
	DeviceNetworker* src;
	float time;
	int content;
    };

    std::list<Message> messagesReceived;
    std::list<Message> messagesToSend;

    DeviceNetworker(); 
    ~DeviceNetworker();

    void connect (DeviceNetworker* r);
    void disconnect (DeviceNetworker* r);
    void connect (Object* o);
    void disconnect (Object* o);

    void send (int content, float time, DeviceNetworker* dst);
    void sendToAll (int content, float time);
    bool receive (Message& msg);

    void actionStep ();
    void perceptionStep ();
    void reset();

    void draw (RenderOpenGL* r);
};


#endif
