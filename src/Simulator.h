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

#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "Object.h"
#include "Service.h"

#include <vector>

class Simulator
{
public : 
    float timestep = 0.1;
    float time = 0.0;

    bool randomizeExecutionSequence;
    
    std::vector<Object*> objects;
    std::vector<Service*> services;

    Simulator(bool silent = false);
    ~Simulator();

    void add (Service* r);
    void add (Object* o);

    void remove (Service* r);
    void remove (Object* o);

    void step ();
    void reset ();
    void setTimestep(float t);
    float getTimestep();
};

#endif
