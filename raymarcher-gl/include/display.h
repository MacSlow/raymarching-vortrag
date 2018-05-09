////////////////////////////////////////////////////////////////////////////////
//
// A desktop-based GLSL-environment similar to the WebGL-based site
// www.shadertoy.com
//
// Copyright 2015-2016 Mirco Müller
//
// Author(s):
//   Mirco "MacSlow" Müller <macslow@gmail.com>
//
// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License version 3, as published
// by the Free Software Foundation.
//
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranties of
// MERCHANTABILITY, SATISFACTORY QUALITY, or FITNESS FOR A PARTICULAR
// PURPOSE.  See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program.  If not, see <http://www.gnu.org/licenses/>.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef _DISPLAY_H
#define _DISPLAY_H

#include <SDL.h>
#include <SDL_image.h>

#include "opengl.h"
#include "file-watch.h"

class Display
{
	public:
      Display (const char* shaderfile, unsigned int width, unsigned int height,
               unsigned int targetFpsRate, const char* imagefile0 = nullptr,
               const char* imagefile1 = nullptr,
               const char* imagefile2 = nullptr,
               const char* imagefile3 = nullptr);
      ~Display ();

      bool run ();

    private:
		bool init ();
		bool resize (unsigned int width, unsigned int height);
		bool update ();		

	private:
		bool _maximized;
		unsigned int _width;
		unsigned int _height;
        unsigned int _targetFpsRate;
        bool _initialized;
        SDL_Window* _window;
        SDL_GLContext _context;
		bool _running;
		bool _paused;
		unsigned int _mouse[2];
		unsigned int _lmbmouse[2];
		OpenGL* _gl;
		FileWatchParameters _file_watch_parameters;
};

#endif // _DISPLAY_H
