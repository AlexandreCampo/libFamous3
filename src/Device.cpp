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

#include "Device.h"

void Device::setPerceptionTimestep(float t)
{
    perceptionTimestep = t;
}

void Device::setActionTimestep(float t)
{
    actionTimestep = t;
}

float Device::getPerceptionTimestep()
{
    return perceptionTimestep;
}

float Device::getActionTimestep()
{
    return actionTimestep;
}

void Device::setObject(Object* o)
{
    this->object = o;
}

Object* Device::getObject()
{
    return object;
}
