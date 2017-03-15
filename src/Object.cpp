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

#include "Object.h"

#include <cstddef>

Object::~Object ()
{
}

void Object::add (Device* d)
{
    devices.push_back(d);
    d->setObject(this);
}

void Object::remove (Device* d)
{
    std::vector<Device*>::iterator it;
    for (it = devices.begin(); it != devices.end(); it++)
    {
	if (*it == d)
	{
	    devices.erase(it);
	    break;
	}
    }
}

void Object::add (Controller* c)
{
    controllers.push_back(c);
    c->object = this;
}

void Object::remove (Controller* c)
{
    std::vector<Controller*>::iterator it;
    for (it = controllers.begin(); it != controllers.end(); it++)
    {
	if (*it == c)
	{
	    (*it)->object = NULL;
	    controllers.erase(it);
	    break;
	}
    }
}

void Object::step ()
{
}

void Object::reset()
{
}

void Object::setTimestep(float t)
{
    timestep = t;
}

float Object::getTimestep()
{
    return timestep;
}
