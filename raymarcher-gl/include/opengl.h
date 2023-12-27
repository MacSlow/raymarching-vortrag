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

#ifndef _OPENGL_H
#define _OPENGL_H

#include <SDL_opengl.h>

#include "texture.h"

class OpenGL
{
	public:
		OpenGL (unsigned int width,
				unsigned int height);
		~OpenGL ();

		bool init (const char* shaderfile = nullptr,
				   const char* imagefile0 = nullptr,
				   const char* imagefile1 = nullptr,
				   const char* imagefile2 = nullptr,
				   const char* imagefile3 = nullptr);
		bool resize (unsigned int width, unsigned int height);
		bool draw (int x, int y, int lmbx, int lmby);
		void reloadShader (const char* shaderfile);

	private:
		friend void checkGLError (const char* func);
		void dumpGLInfo ();
		GLuint loadShader (const char *src, GLenum type);
		GLuint createShaderProgram (const char* vertSrc,
									const char* fragSrc,
									bool link);
		void linkShaderProgram (GLuint progId);
		void dumpProgramBinaryInfoToConsole (GLuint progId) const;

	private:
		unsigned int _width;
		unsigned int _height;
		unsigned int _frame;
        Texture* _texture[4];
		GLuint _vShaderId;
		GLuint _fShaderId;
		GLuint _program;
		GLuint _vao;
		GLuint _vbo;
		GLint _iResolution;
		GLint _iGlobaltime;
		GLint _iChannelRes[4];
		GLint _iFrame;
		GLint _iMouse;
		GLint _iChannel[4];
		GLint _iDate;
};

#endif // _OPENGL_H
