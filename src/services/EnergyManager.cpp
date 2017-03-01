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

	// // transfer energy
	// float src = transactions[i].src->energyLevel - transactions[i].energy;
	// float dst = transactions[i].dst->energyLevel + transactions[i].energy;

	// if (src < 0.0) {dst += src; src = 0.0;}
	// if (dst < 0.0) {src += dst; dst = 0.0;}

	// float diffsrc = src - transactions[i].src->energyCapacity;
	// if (diffsrc > 0.0) 
	// {
	//     dst += diffsrc;
	//     src = transactions[i].src->energyCapacity;
	// }

	// float diffdst = dst - transactions[i].dst->energyCapacity;
	// if (diffdst > 0.0) 
	// {
	//     src += diffdst;
	//     dst = transactions[i].dst->energyCapacity;
	// }

	// transactions[i].dst->energyLevel = dst;
	// transactions[i].src->energyLevel = src;

	// todo test : reject transaction if not implementable as is

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
//	    std::cout << "accept : " << e << "\t" << transactions[i].src->energyLevel << "\t" << transactions[i].dst->energyLevel << "\t\t" << transactions[i].src->energyLevel-e << "\t" << transactions[i].dst->energyLevel+e <<std::endl;

	    transactions[i].dst->energyLevel += e;
	    transactions[i].src->energyLevel -= e;
	    
	    transactions[i].dst->processed = true;
	    transactions[i].src->processed = true;	    
	}
	else
	{
//	    std::cout << "reject : " << e << "\t" << transactions[i].src->energyLevel << "\t" << transactions[i].dst->energyLevel << std::endl;
	}

	// if (transactions[i].dst->energyLevel < 0.96 && transactions[i].dst->energyLevel > 0.94)
	// {
	//     std::cout << "Weird stuff on : " << e << " " << transactions[i].src->energyLevel << " " << transactions[i].dst->energyLevel << std::endl;
	// }
	// if (transactions[i].src->energyLevel < 0.96 && transactions[i].src->energyLevel > 0.94)
	// {
	//     std::cout << "Weird stuff on : " << e << " " << transactions[i].src->energyLevel << " " << transactions[i].dst->energyLevel << std::endl;
	// }

	// float e;
	// if (transactions[i].energy <= transactions[i].src->energyLevel) 
	//     e = transactions[i].energy;
	// else
	//     e = transactions[i].src->energyLevel;

	// if (e > transactions[i].dst->energyCapacity - transactions[i].dst->energyLevel)
	//     e = transactions[i].dst->energyCapacity - transactions[i].dst->energyLevel;
	
	// transactions[i].dst->energyLevel += e;
	// transactions[i].src->energyLevel -= e;
    }
    
    transactions.clear();
}

void EnergyManager::Reset ()
{
    transactions.clear();
}


