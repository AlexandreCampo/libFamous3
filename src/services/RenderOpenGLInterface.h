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

#ifndef RENDER_OPENGL_INTERFACE_H
#define RENDER_OPENGL_INTERFACE_H

#include "RenderOpenGL.h"

#include <GL/glut.h> 
#include <GL/gl.h>   
#include <GL/glu.h>  

class RenderOpenGLInterface 
{
public : 

    virtual void Register (RenderOpenGL* r);
    virtual void Unregister (RenderOpenGL* r);    

    virtual void Draw(RenderOpenGL* r) = 0;
};

#endif
