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


#include "DeviceGSM.h"
#include "Object.h"
#include "GSMNetworkInterface.h"
#include "Simulator.h"

#include <list>
#include <vector>
#include <iostream>


DeviceGSM::DeviceGSM(GSMNetwork* n)
{
    this->gsmNetwork = n;
}

void DeviceGSM::actionStep ()
{
}

void DeviceGSM::perceptionStep ()
{
}
    
void DeviceGSM::reset ()
{
    GSMNetworkInterface* o = (GSMNetworkInterface*) object;

    if (o) o->GSMNetworkInterface::messagesReceived.clear();
}

void DeviceGSM::send (int dst, std::vector<char> content)
{
    GSMNetworkInterface* o = (GSMNetworkInterface*) object;

    gsmNetwork->messagesInTransit.push_back(GSMNetwork::Message(o->id, dst, object->simulator->time, content));
}

bool DeviceGSM::receive (GSMNetwork::Message& msg)
{
    GSMNetworkInterface* o = (GSMNetworkInterface*) object;
    
    if (o->GSMNetworkInterface::messagesReceived.empty())
    {	
	return false;
    }
    else
    {
	msg = o->GSMNetworkInterface::messagesReceived.front();
	o->GSMNetworkInterface::messagesReceived.pop_front();
    }
    return true;
}

void DeviceGSM::setObject(Object* o)
{
    // downcast
    object = (Object*) dynamic_cast<GSMNetworkInterface*> (o);
    
    if (!object)
	std::cerr << "DeviceGSM was added to an object that does not implement GSMNetworkInterface !" << std::endl;
}
