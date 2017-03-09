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
#include "Simulator.h"


#include <unistd.h>

#include <cmath>
#include <cstring>
#include <iostream>
#include <string>
#include <fstream>
#include <sys/time.h>
#include <algorithm>

#include "EnergyManager.h"


void EnergyManager::Step ()
{
    // go through the list of objects and mark them as unprocessed
    for (unsigned int i = 0; i < objects.size(); i++)
    {
	objects[i]->processed = false;
    }

    // randomize transactions and process them. Some transactions may not be possible...
    std::random_shuffle (transactions.begin(), transactions.end());

    for (unsigned int i = 0; i < transactions.size(); i++)
    {
	bool reject = false;
	float e = transactions[i].energy;

	if (transactions[i].src->processed)
	    reject = true;

	else if (transactions[i].dst->processed)
	    reject = true;

	else if (e > transactions[i].src->energyLevel) 
	    reject = true;

	else if (e > transactions[i].dst->energyCapacity - transactions[i].dst->energyLevel)
	    reject = true;

	
	if (!reject)
	{
	    transactions[i].dst->energyLevel += e;
	    transactions[i].src->energyLevel -= e;
	    
	    transactions[i].dst->processed = true;
	    transactions[i].src->processed = true;	    
	}
    }
    
    transactions.clear();
}

void EnergyManager::Reset ()
{
    transactions.clear();
}


