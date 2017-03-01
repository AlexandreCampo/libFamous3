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

#include "RenderOSGInterface.h"

#include <list>


void RenderOSGInterface::Register (RenderOSG* r)
{
    renderOSG = r;
    r->objects.push_back(this);
}

void RenderOSGInterface::Unregister (RenderOSG* r)
{
    std::list<RenderOSGInterface*>::iterator it;
    for (it = r->objects.begin(); it != r->objects.end(); it++)
    {
    	if (*it == this)
    	{
    	    r->objects.erase(it);
    	}
    }
    renderOSG = NULL;
}
