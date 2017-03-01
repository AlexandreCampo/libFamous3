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


#include "DeviceNetworker.h"
#include "RenderOpenGLInterface.h"
#include "Object.h"

#include <list>
#include <vector>
#include <iostream>
#include <typeinfo>



DeviceNetworker::DeviceNetworker()
{
    Reset();
}

DeviceNetworker::~DeviceNetworker()
{

}

void DeviceNetworker::ActionStep ()
{
    std::list<Message>::iterator it = messagesToSend.begin();
    for (; it != messagesToSend.end(); ++it)
    {
	if (it->dst != NULL)
	{
	    it->dst->messagesReceived.push_back(*it);
	}
	else
	{
	    std::list<DeviceNetworker*>::iterator it2 = connections.begin();
	    for (; it2 != connections.end(); ++it2)
	    {
		(*it2)->messagesReceived.push_back(*it);		
	    }
	}	
    }
    
    // all messages have been sent
    messagesToSend.clear();
}

void DeviceNetworker::PerceptionStep ()
{
    // everything is done in action step...
}
    
void DeviceNetworker::Reset ()
{
    messagesReceived.clear();
    messagesToSend.clear();
}

void DeviceNetworker::Draw (RenderOpenGL* r)
{   
}


void DeviceNetworker::Send (int content, float time, DeviceNetworker* dst)
{
    Message m (content, time, this, dst);
    messagesToSend.push_back(m);
}

void DeviceNetworker::SendToAll (int content, float time)
{
    Message m (content, time, this, NULL);
    messagesToSend.push_back(m);
}

bool DeviceNetworker::Receive (Message& msg)
{
    if (messagesReceived.empty())
    {	
	return false;
    }
    else
    {
	msg = messagesReceived.front();
	messagesReceived.pop_front();
    }
}


void DeviceNetworker::Connect (Object* o)
{
    // find the device of the object
    for (unsigned int i = 0; i < o->devices.size(); i++)
    {
	if (typeid (*(o->devices[i])) == typeid(*this))
	{
	    Connect (static_cast<DeviceNetworker*> (o->devices[i]));
	}
    }
}

void DeviceNetworker::Disconnect (Object* o)
{
    // find the device of the object
    for (unsigned int i = 0; i < o->devices.size(); i++)
    {
	if (typeid (*(o->devices[i])) == typeid(*this))
	{
	    Disconnect (static_cast<DeviceNetworker*> (o->devices[i]));
	}
    }
}

void DeviceNetworker::Connect (DeviceNetworker* r)
{
    connections.push_back(r);
}

void DeviceNetworker::Disconnect (DeviceNetworker* r)
{
    std::list<DeviceNetworker*>::iterator it;
    for (it = connections.begin(); it != connections.end(); it++)
    {
	if (*it == r)
	{
	    connections.erase(it);
	}
    }    
}
