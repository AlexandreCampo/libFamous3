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


#ifndef WATER_VOLUME_H
#define WATER_VOLUME_H

#include "Service.h"

#include <functional>
#include "LinearMath/btVector3.h"

class WaterVolumeInterface;

typedef std::function< float (btVector3, float) > WaterVolumeHeightCallback;
typedef std::function< btVector3 (btVector3, float) > WaterVolumeCurrentCallback;

class WaterVolume : public Service
{
public :

    // callbacks that should be implemented by user
    WaterVolumeHeightCallback calculateHeight;
    WaterVolumeCurrentCallback calculateCurrent;

    float density;
    
    // Methods ==================

    WaterVolume(float density, WaterVolumeHeightCallback calculateHeightCallback, WaterVolumeCurrentCallback calculateCurrentCallback);
    ~WaterVolume();

    void step ();
    void reset ();
};


#endif



