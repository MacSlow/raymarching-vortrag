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
#include <SDL.h>
#include <SDL_image.h>
#include <GL/glew.h>

#include "texture.h"

Texture::Texture (const char* file,
				  GLenum unit,
				  GLint channelResId,
				  GLint channelId,
				  GLint channel) :
	_unit (unit),
	_channelResId (channelResId),
	_channelId (channelId),
	_channel (channel)
{
	_id = loadTexture (file, &_width, &_height);
}

Texture::~Texture ()
{
    glDeleteTextures (1, &_id);
}

void Texture::use () const
{
	if (_id != 0) {
		glActiveTexture (_unit);
		glBindTexture (GL_TEXTURE_2D, _id);
		glUniform3f (_channelResId, (GLfloat) _width, (GLfloat) _height, 0.0);
		glUniform1i (_channelId, _channel);		
	}
}

GLuint Texture::loadTexture (const char* filename,
                             GLuint* width,
                             GLuint* height)
{
    if (!filename) {
        return 0;
    }

    GLuint texture = 0;
    SDL_Surface* surface = NULL;
    surface = IMG_Load (filename);
    if (!surface) {
        std::cout << "Failed to create texture: " << IMG_GetError() << std::endl;
        return 0;
    }

    SDL_Surface* converted = NULL;
    converted = SDL_ConvertSurfaceFormat (surface, SDL_PIXELFORMAT_ABGR8888, 0);
    SDL_FreeSurface (surface);

    if (width) {
        *width = converted->w;
    }
    if (height) {
        *height = converted->h;
    }

    glGenTextures (1, &texture);
    glBindTexture (GL_TEXTURE_2D, texture);
    glTexImage2D (GL_TEXTURE_2D,
                  0,
                  GL_RGBA,
                  converted->w,
                  converted->h,
                  0,
                  GL_RGBA,
                  GL_UNSIGNED_BYTE,
                  converted->pixels);
    SDL_FreeSurface (converted);
    glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri (GL_TEXTURE_2D,
                     GL_TEXTURE_MIN_FILTER,
                     GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glGenerateMipmap (GL_TEXTURE_2D);
    glBindTexture (GL_TEXTURE_2D, 0);

    return texture;
}
