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

#ifndef OBJECT_H
#define OBJECT_H

#include <vector>

#include "Service.h"
#include "Device.h"
#include "Controller.h"

#include <limits>

class Simulator;

class Object
{
public:

    Simulator* simulator;
    std::vector<Service*> services;
    std::vector<Device*> devices;
    std::vector<Controller*> controllers;

    float timestep = std::numeric_limits<float>::max();;
    float nextTime = 0.0;
    
    virtual ~Object();

    virtual void add (Device* device);
    virtual void remove (Device* d);
    virtual void add (Controller* controller);
    virtual void remove (Controller* c);
    virtual void step ();
    virtual void reset ();

    void setTimestep(float t);
    float getTimestep();    
};

#endif
