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

#include <iostream>
#include <string>
#include <sstream>

#include "config.h"
#include "display.h"

#define WIN_TITLE "GL-Raymarcher by MacSlow"

bool Display::init ()
{
	if (_initialized) {
		return false;
	}

	// initialize SDL
	int result = 0;
	SDL_ClearError ();
	result = SDL_Init (SDL_INIT_VIDEO);
	if (result != 0) {
		std::cout << "SDL_Init() failed: " << SDL_GetError () << std::endl;
		_initialized = false;
		return _initialized;
	}

    // initialize SDL_image
    int flags = IMG_INIT_PNG | IMG_INIT_JPG;
	SDL_ClearError ();
    result = IMG_Init (flags);
    if ((result & flags) != flags) {
        std::cout << "IMG_Init() failed: " << IMG_GetError () << std::endl;
        SDL_Quit ();
		_initialized = false;
		return _initialized;
    }

    _initialized = true;
    return _initialized;
}

Display::Display (const char* shaderfile,
				  unsigned int width,
				  unsigned int height,
				  const char* imagefile0,
				  const char* imagefile1,
				  const char* imagefile2,
				  const char* imagefile3) :
	_maximized (false),
	_width (width),
	_height (height),
	_initialized (false),
	_window (NULL),
	_running (false),
	_paused (false),
	_mouse {0, 0},
	_lmbmouse {0, 0},
	_file_watch_parameters {nullptr, 0}
{
	init ();

	_file_watch_parameters.filename = std::make_shared<std::string> (shaderfile);
	update_checksum (&_file_watch_parameters);

    SDL_GL_SetAttribute (SDL_GL_RED_SIZE, 8);
    SDL_GL_SetAttribute (SDL_GL_GREEN_SIZE, 8);
    SDL_GL_SetAttribute (SDL_GL_BLUE_SIZE, 8);
    SDL_GL_SetAttribute (SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute (SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute (SDL_GL_MULTISAMPLESAMPLES, 4);
    SDL_GL_SetAttribute (SDL_GL_CONTEXT_PROFILE_MASK,
						 SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute (SDL_GL_CONTEXT_PROFILE_MASK,
						 SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute (SDL_GL_CONTEXT_MAJOR_VERSION, 4);
    SDL_GL_SetAttribute (SDL_GL_CONTEXT_MINOR_VERSION, 1);

	SDL_ClearError ();
	_window = SDL_CreateWindow (WIN_TITLE,
								SDL_WINDOWPOS_UNDEFINED,
								SDL_WINDOWPOS_UNDEFINED,
								_width,
								_height,
								SDL_WINDOW_OPENGL |
								SDL_WINDOW_RESIZABLE);
	if (!_window) {
		std::cout << "window creation failed: " << SDL_GetError () << std::endl;
		return;
	}

	SDL_Surface* icon = IMG_Load (DATA_DIR"/icon.png");
	SDL_SetWindowIcon (_window, icon);
	SDL_FreeSurface (icon);

	// setup OpenGL-context
	SDL_ClearError ();
	_context = SDL_GL_CreateContext (_window);
	if (!_context) {
		std::cout << "CreateContext() failed: " << SDL_GetError () << std::endl;
		SDL_DestroyWindow (_window);
        IMG_Quit ();
		SDL_Quit ();
		_initialized = false;
		return;
	}

	_gl = new OpenGL (_width, _height);
	_gl->init (shaderfile, imagefile0, imagefile1, imagefile2, imagefile3);
}

Display::~Display ()
{
	delete _gl;
	SDL_GL_DeleteContext (_context);
	SDL_DestroyWindow (_window);
	IMG_Quit ();
	SDL_Quit ();
}

bool Display::run ()
{
	if (!_initialized) {
		return false;
	}

	_running = true;

	while (_running) {
		SDL_Event event;
		while (SDL_PollEvent (&event)) {
			switch (event.type) {	
				case SDL_KEYUP:
					if (event.key.keysym.sym == SDLK_ESCAPE) {
						_running = false;
					} else if (event.key.keysym.sym == SDLK_F11) {
						if (_maximized) {
							SDL_SetWindowFullscreen (_window, 0);
							_maximized = false;
						} else {
							SDL_SetWindowFullscreen (
								_window,
								SDL_WINDOW_FULLSCREEN_DESKTOP);
							_maximized = true;
						}
					} else if (event.key.keysym.sym == SDLK_SPACE) {
						_paused = !_paused;
					}
				break;

				case SDL_MOUSEMOTION:
					_mouse[0] = event.motion.x;
					_mouse[1] = event.motion.y;
					if (SDL_GetMouseState(NULL, NULL) &
						SDL_BUTTON (SDL_BUTTON_LEFT)) {
						_lmbmouse[0] = event.button.x;
						_lmbmouse[1] = event.button.y;					
					}
				break;

				case SDL_WINDOWEVENT:
					if (event.window.event == SDL_WINDOWEVENT_CLOSE) {
						_running = false;
					} else if (event.window.event == SDL_WINDOWEVENT_RESIZED) {
						resize (event.window.data1,
								event.window.data2);
					}
				break;

				case SDL_QUIT:
					_running = false;
				break;
			}
		}

		update ();
		SDL_Delay (1000/60);
	}

	return true;
}

bool Display::resize (unsigned int width, unsigned int height)
{
	_width = width;
	_height = height;
	_gl->resize (_width, _height);

	return true;
}

bool Display::update ()
{
	if (!_initialized) {
		return false;
	}

	if (_paused) {
        std::stringstream title;
        title << WIN_TITLE << " - paused";
        std::string str (title.str ());
        SDL_SetWindowTitle (_window, str.c_str ());
		return true;
	}

	_gl->draw (_mouse[0], _mouse[1], _lmbmouse[0], _lmbmouse[1]);
	SDL_GL_SwapWindow (_window);

    static unsigned int fps = 0;
    static unsigned int lastTick = 0;
    static unsigned int currentTick = 0;

    // spit out frame-rate and frame-time
    fps++;
    currentTick = SDL_GetTicks ();
    if (currentTick - lastTick > 1000) {
        std::stringstream title;
        title << WIN_TITLE << " - " << fps << " fps";
        std::string str (title.str ());
        SDL_SetWindowTitle (_window, str.c_str ());
        fps = 0;
        lastTick = currentTick;

		if (did_file_change (_file_watch_parameters.filename,
							 _file_watch_parameters.checksum)) {
			update_checksum (&_file_watch_parameters);
			_gl->reloadShader (_file_watch_parameters.filename->c_str());
		}
    }

	return true;
}
