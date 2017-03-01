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

#include "EnergyManagerInterface.h"

#include <list>

EnergyManagerInterface::EnergyManagerInterface (float capacity, float level)
{
    this->energyLevel = level;
    this->energyCapacity = capacity;
}

void EnergyManagerInterface::Register (EnergyManager* m)
{
    energyManager = m;
    m->objects.push_back(this);
}

void EnergyManagerInterface::Unregister (EnergyManager* m)
{
    std::vector<EnergyManagerInterface*>::iterator it;
    for (it = m->objects.begin(); it != m->objects.end(); it++)
    {
	if (*it == this)
	{
	    m->objects.erase(it);
	}
    }
    energyManager = NULL;
}

void EnergyManagerInterface::Connect (EnergyManagerInterface* o)
{
    energyConnections.push_back(o);
}

void EnergyManagerInterface::Disconnect (EnergyManagerInterface* o)
{
    std::list<EnergyManagerInterface*>::iterator it;
    for (it = energyConnections.begin(); it != energyConnections.end(); it++)
    {
	if (*it == o)
	{
	    energyConnections.erase(it);
	}
    }    
}

// void EnergyManagerInterface::Transfer (EnergyManagerInterface* dst, float energy)
// {
//     energyManager->transactions.push_back(EnergyManager::Transaction(this, dst, energy));
// }

void EnergyManagerInterface::Push (EnergyManagerInterface* dst, float energy)
{
    energyManager->transactions.push_back(EnergyManager::Transaction(this, dst, energy));
}

void EnergyManagerInterface::Pull (EnergyManagerInterface* src, float energy)
{
    energyManager->transactions.push_back(EnergyManager::Transaction(src, this, energy));
}
