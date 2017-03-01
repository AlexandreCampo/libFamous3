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


#ifndef GSM_NETWORK_H
#define GSM_NETWORK_H

#include <vector>
#include <map>

#include "Service.h"

class GSMNetworkInterface;

class GSMNetwork : public Service
{
public :
    struct Message
    {
	int src;
	int dst;
	float time;
	std::vector<char> content;

	Message (int src, int dst, float time, std::vector<char> content)
	    {
		this->src = src;
		this->dst = dst;
		this->time = time;
		this->content = content;
	    }
    };

    // directly store in a tree the objects, associated to their ID number
    std::map<int, GSMNetworkInterface*> objects;
    std::vector<Message> messagesInTransit;
    
       
    // Methods ==================
    GSMNetwork();
    ~GSMNetwork();

    void Step ();
    void Reset ();
};


#endif



