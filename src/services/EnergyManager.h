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


#ifndef ENERGY_MANAGER_H
#define ENERGY_MANAGER_H

#include <cmath>
#include <list>
#include <vector>
#include <sys/time.h>
#include <string>

#include "Service.h"

class EnergyManagerInterface;

class EnergyManager : public Service
{
public :
    struct Transaction
    {
	float energy;
	EnergyManagerInterface* src;
	EnergyManagerInterface* dst;

	Transaction (EnergyManagerInterface* src, EnergyManagerInterface* dst, float energy)
	    {
		this->energy = energy;
		this->src = src;
		this->dst = dst;
	    }
    };

    std::vector<EnergyManagerInterface*> objects;
    std::vector<Transaction> transactions;

       
    // Methods ==================

    EnergyManager();
    ~EnergyManager();

    void Step ();
    void Reset ();
};


#endif



