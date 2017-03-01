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

#include "Simulator.h"
#include "RenderOpenGLInterface.h"

#include <iostream>
#include <algorithm>


Simulator::Simulator(bool silent) 
{
    if (!silent)
    {
	// the simulator is created : display an advertising banner 
	std::cout << "                                                                               " << std::endl;
	std::cout << "                                                                               " << std::endl;
	std::cout << "                                                                               " << std::endl;
	std::cout << "88888888888          88b           d88                             ad88888ba   " << std::endl;
	std::cout << "88                   888b         d888                            d8\"     \"8b  " << std::endl;
	std::cout << "88                   88`8b       d8'88                            Y8,          " << std::endl;
	std::cout << "88aaaaa  ,adPPYYba,  88 `8b     d8' 88   ,adPPYba,   88       88  `Y8aaaaa,    " << std::endl;
	std::cout << "88\"\"\"\"\"  \"\"     `Y8  88  `8b   d8'  88  a8\"     \"8a  88       88    `\"\"\"\"\"8b,  " << std::endl;
	std::cout << "88       ,adPPPPP88  88   `8b d8'   88  8b       d8  88       88          `8b  " << std::endl;
	std::cout << "88       88,    ,88  88    `888'    88  \"8a,   ,a8\"  \"8a,   ,a88  Y8a     a8P  " << std::endl;
	std::cout << "88       `\"8bbdP\"Y8  88     `8'     88   `\"YbbdP\"'    `\"YbbdP'Y8   \"Y88888P\"   " << std::endl;
	std::cout << "                                                                               " << std::endl;
	std::cout << "                                        A Fast, Modular, and Simple simulator  " << std::endl;         
	std::cout << "                                                                               " << std::endl;
//    std::cout << "                                                                               " << std::endl;
    }

    time = 0.0;
    timestep = 0.05;
    randomizeExecutionSequence = false;
}

Simulator::~Simulator()
{
    for (unsigned int i = 0; i < objects.size(); i++)
    {
	delete objects[i];
    }

    for (unsigned int i = 0; i < services.size(); i++)
    {
	delete services[i];
    }
}

void Simulator::Add (Service* s)
{
    s->simulator = this;
    services.push_back(s);
}

void Simulator::Add (Object* o)
{
    o->simulator = this;
    objects.push_back(o);
}

void Simulator::Remove (Service* s)
{
    std::vector<Service*>::iterator it;
    for (it = services.begin(); it != services.end(); it++)
    {
	if (*it == s)
	{
	    it = services.erase(it);
	    break;
	}
    }
}

void Simulator::Remove (Object* o)
{
    std::vector<Object*>::iterator it;
    for (it = objects.begin(); it != objects.end(); it++)
    {
	if (*it == o)
	{
	    it = objects.erase(it);
	    break;
	}
    }
}

void Simulator::Step ()
{    
    // randomize execution order of objects 
    if (randomizeExecutionSequence)
    {
	std::random_shuffle (objects.begin(), objects.end());	
    }
    
    // for each object
    // first perceive the environment,
    for (auto* o : objects)
    {
	for (auto* d : o->devices)
	{
	    if (time >= d->nextPerceptionTime)
	    {
		d->PerceptionStep();
		d->nextPerceptionTime += d->perceptionTimestep;
	    }
	}
    }

    // then controllers of objects process information
    for (auto* o : objects)
    {
	// controller
	if (o->controller)
	{
	    if (time >= o->controller->nextTime)
	    {
		o->controller->Step();
		o->controller->nextTime += o->controller->timestep;
	    }	    
	}
    }

    // then execute some actions in the environment    
    for (auto* o : objects)
    {
	for (auto* d : o->devices)
	{
	    if (time >= d->nextActionTime)
	    {
		d->ActionStep();
		d->nextActionTime += d->actionTimestep;
	    }
	}
    }

    // finally objects themselves can also do some actions
    for (auto* o : objects)
    {
	if (time >= o->nextTime)
	{
	    o->Step();
	    o->nextTime += o->timestep;
	}
    }
    
    // for each service
    // this call must happen here, because services may provide additional information to devices 
    // (eg bullet uses callback methods for collision detection)
    for (auto* s : services)
    {
	if (time >= s->nextTime)
	{
	    s->Step();
	    s->nextTime += s->timestep;
	}
    }

    time += timestep;
}

void Simulator::Reset ()
{
    time = 0.0;

    // it makes sense to reset in this central point that will anyway
    // also call the experiment. Hence any service can ask for a reset, without
    // knowing who does what, so this leaves freedom in what happens.
    for (auto* s : services)
    {
    	s->Reset();
	s->nextTime = 0;
    }

    for (auto* o : objects)
    {
    	o->Reset();
    	o->nextTime = 0;

    	if (o->controller != NULL)
    	{
    	    o->controller->Reset();
    	    o->controller->nextTime = 0;
    	}
	
    	for (auto* d : o->devices)
    	{
    	    d->Reset();
    	    d->nextPerceptionTime = 0;
    	    d->nextActionTime = 0;
    	}
    }
}

void Simulator::SetTimeStep(float t)
{
    timestep = t;
}

float Simulator::GetTimeStep()
{
    return timestep;
}
