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

#ifndef _TEXTURE_H
#define _TEXTURE_H

#include <SDL_opengl.h>

class Texture
{
	public:
		Texture (const char* file,
				 GLenum unit,
				 GLint channelRes,
				 GLint channelId,
				 GLint channel);
		~Texture ();

    	void use () const;

    private:
		GLuint loadTexture (const char* filename,
							GLuint* width,
							GLuint* height);

	private:
		GLenum _unit;
		GLuint _id;
		GLint _channelResId;
		GLuint _width;
		GLuint _height;
		GLint _channelId;
		GLint _channel;
};

#endif // _TEXTURE_H
