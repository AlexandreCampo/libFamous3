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

#ifndef ENERGY_MANAGER_INTERFACE_H
#define ENERGY_MANAGER_INTERFACE_H

#include "EnergyManager.h"

#include <list>


class EnergyManagerInterface 
{
public : 
    EnergyManager* energyManager;

    float energyCapacity;
    float energyLevel;
    bool processed;
    std::list<EnergyManagerInterface*> energyConnections;

    EnergyManagerInterface (float capacity, float level);

    virtual void Register (EnergyManager* r);
    virtual void Unregister (EnergyManager* r);    

    void Connect (EnergyManagerInterface* o);
    void Disconnect (EnergyManagerInterface* o);

    void Push (EnergyManagerInterface* o, float energy);
    void Pull (EnergyManagerInterface* o, float energy);
//    void Transfer (EnergyManagerInterface* o, float energy);
};

#endif
