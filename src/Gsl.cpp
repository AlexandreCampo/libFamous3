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

#include "Gsl.h"


// Functions for gsl random numbers
gsl_rng* rng;
long int rngSeed;

void init_rng(gsl_rng** RNG)
{
    // init random numbers
    *RNG = gsl_rng_alloc (gsl_rng_taus);

    // reset random number generator
    struct timeval tv;
    gettimeofday(&tv, 0);
    srandom(tv.tv_sec ^ tv.tv_usec);
//    srand(tv.tv_sec ^ tv.tv_usec);
    rngSeed = random();
    gsl_rng_set (*RNG, rngSeed);
}

void del_rng(gsl_rng* RNG)
{
    // delete random number generator
    gsl_rng_free (RNG);
}

void set_seed_rng(gsl_rng* RNG, long int seed)
{
    gsl_rng_set (RNG, seed);
}

