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

#ifndef DEVICE_H
#define DEVICE_H

class Object;

#include <limits>

class Device
{
public :
    Object* object = nullptr;
   
    float perceptionTimestep = std::numeric_limits<float>::max();
    float actionTimestep = std::numeric_limits<float>::max();
    float nextPerceptionTime = 0.0;
    float nextActionTime = 0.0;

    virtual void PerceptionStep () = 0;
    virtual void ActionStep () = 0;
    virtual void Reset () = 0;

    void SetPerceptionTimeStep(float t);
    void SetActionTimeStep(float t);
    float GetPerceptionTimeStep();    
    float GetActionTimeStep();    
};

#endif
