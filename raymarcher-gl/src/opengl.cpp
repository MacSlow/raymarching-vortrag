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

// the uniform inputs taken over from shadertoy.com
// uniform vec3      iResolution (width, height, aspect)
// uniform float     iGlobalTime (time in seconds since program start)
// uniform vec3      iChannelResolution0..3 (texture-resolution for each unit)
// uniform vec4      iMouse (xy = pixel-coords, zw = LMB-clicked pixel-coords)
// uniform sampler2D iChannel0..3 (texture-data for units 0..3)
// uniform vec4      iDate (x = year, y = month, z = day, w = daytime-seconds)

#include <iostream>
#include <cassert>
#include <list>
#include <sstream>
#include <iterator>
#include <ctime>
#include <iomanip>
#include <fstream>
#include <iostream>
#include <string>

#include <SDL.h>
#include <SDL_opengl.h>

#include "opengl.h"
#include "shaders.h"

enum VertexAttribs {
    PositionAttr = 0,
    TexCoordAttr,
    NormalAttr
};

OpenGL::OpenGL (unsigned int width,
				unsigned int height) :
	_width (width),
	_height (height),
    _texture {nullptr, nullptr, nullptr, nullptr},
	_vShaderId (0),
	_fShaderId (0),
	_program (0),
	_vao (0),
	_vbo (0),
	_iResolution (0),
	_iGlobaltime (0),
	_iChannelRes {0, 0, 0, 0},
	_iMouse (0),
	_iChannel {0, 0, 0, 0},
	_iDate (0)
{
}

OpenGL::~OpenGL ()
{
    delete _texture[0];
    delete _texture[1];
    delete _texture[2];
    delete _texture[3];
	glDeleteProgram (_program);
	glDeleteShader (_vShaderId);
	glDeleteShader (_fShaderId);
	glDeleteVertexArrays (1, &_vao);
	glDeleteBuffers (1, &_vbo);
}

bool OpenGL::init (const char* shaderfile,
				   const char* imagefile0,
                   const char* imagefile1,
                   const char* imagefile2,
                   const char* imagefile3)
{
	dumpGLInfo ();
    glClearColor (1.0, 1.0, 1.0, 1.0);
    glViewport (0, 0, _width, _height);
    glEnable (GL_BLEND);

    std::string shader = "";
    if (shaderfile) {
    	std::ifstream file (shaderfile);
	    if (file) {
			std::string line;
			while (!std::getline (file, line).eof ())
				shader += line + '\n';	    	
	    }
    }

    _program = createShaderProgram (vert, shader.c_str(), true);

    // the supported ShaderToy-like uniforms
	_iResolution = glGetUniformLocation (_program, "iResolution");
	_iGlobaltime = glGetUniformLocation (_program, "iTime");
	_iChannelRes[0] = glGetUniformLocation (_program, "iChannelResolution0");
	_iChannelRes[1] = glGetUniformLocation (_program, "iChannelResolution1");
	_iChannelRes[2] = glGetUniformLocation (_program, "iChannelResolution2");
	_iChannelRes[3] = glGetUniformLocation (_program, "iChannelResolution3");
	_iMouse = glGetUniformLocation (_program, "iMouse");
	_iChannel[0] = glGetUniformLocation (_program, "iChannel0");
	_iChannel[1] = glGetUniformLocation (_program, "iChannel1");
	_iChannel[2] = glGetUniformLocation (_program, "iChannel2");
	_iChannel[3] = glGetUniformLocation (_program, "iChannel3");
	_iDate = glGetUniformLocation (_program, "iDate");

    // load any potential texture-images
    if (imagefile0) {
        _texture[0] = new Texture (imagefile0,
                                   GL_TEXTURE0,
                                   _iChannelRes[0],
                                   _iChannel[0],
                                   0);
    }

    if (imagefile1) {
        _texture[1] = new Texture (imagefile1,
                                   GL_TEXTURE1,
                                   _iChannelRes[1],
                                   _iChannel[1],
                                   1);
    }

    if (imagefile2) {
        _texture[2] = new Texture (imagefile2,
                                   GL_TEXTURE2,
                                   _iChannelRes[2],
                                   _iChannel[2],
                                   2);
    }

    if (imagefile3) {
        _texture[3] = new Texture (imagefile3,
                                   GL_TEXTURE3,
                                   _iChannelRes[3],
                                   _iChannel[3],
                                   3);
    }

	glGenVertexArrays (1, &_vao);
	glGenBuffers (1, &_vbo);

	GLfloat quad[16] = {-1.0, -1.0,
						 0.0,  0.0,
						 1.0, -1.0,
						 1.0,  0.0,
						 1.0,  1.0,
						 1.0,  1.0,
						-1.0,  1.0,
						 0.0,  1.0};
	glBindVertexArray (_vao);
	glBindBuffer (GL_ARRAY_BUFFER, _vbo);
	glBufferData (GL_ARRAY_BUFFER, 16 * sizeof (GLfloat), quad, GL_STATIC_DRAW);
	glBindAttribLocation (_program, PositionAttr, "aPosition");
	glBindAttribLocation (_program, TexCoordAttr, "aTexCoord");
	GLchar* offset = 0;
    glVertexAttribPointer (PositionAttr,
                           2,
                           GL_FLOAT,
                           GL_FALSE,
                           4 * sizeof (GLfloat),
                           offset);
	glEnableVertexAttribArray (PositionAttr);
    glVertexAttribPointer (TexCoordAttr,
                           2,
                           GL_FLOAT,
                           GL_FALSE,
                           4 * sizeof (GLfloat),
                           2 * sizeof (GLfloat) + offset);
	glEnableVertexAttribArray (TexCoordAttr);

	glBindVertexArray (0);
	glBindBuffer (GL_ARRAY_BUFFER, 0);

	return true;
}

bool OpenGL::resize (unsigned int width, unsigned int height)
{
	_width = width;
	_height = height;

	glViewport (0, 0, _width, _height);

	return true;
}

bool OpenGL::draw (int x, int y, int lmbx, int lmby)
{
	glClear (GL_COLOR_BUFFER_BIT);

	glUseProgram (_program);
    glBindVertexArray (_vao);

    // set the supported ShaderToy-like uniforms
    if (_texture[0]) {
        _texture[0]->use ();
    }
    if (_texture[1]) {
        _texture[1]->use ();
    }
    if (_texture[2]) {
        _texture[2]->use ();
    }
    if (_texture[3]) {
        _texture[3]->use ();
    }

	glUniform3f (_iResolution,
				 (GLfloat) _width,
				 (GLfloat) _height,
				 (GLfloat) _width / (GLfloat) _height);
	glUniform1f (_iGlobaltime, (GLfloat) SDL_GetTicks () / 1000.0);
	glUniform4f (_iMouse,
				 (GLfloat) x,
				 (GLfloat) y,
				 (GLfloat) lmbx,
				 (GLfloat) lmby);
 
 	std::time_t rawt = std::time (nullptr);
    struct tm* t = std::localtime (&rawt);

	glUniform4f (_iDate,
				 (GLfloat) t->tm_year,
				 (GLfloat) t->tm_mon,
				 (GLfloat) t->tm_mday,
				 (GLfloat) (3600 * t->tm_hour + 60 * t->tm_min + t->tm_sec));

	static unsigned short indices[6] = {0, 1, 2, 3, 2, 0};
	glDrawElements (GL_TRIANGLES, 6, GL_UNSIGNED_SHORT, indices);

    glBindVertexArray (0);
    glUseProgram (0);

	return true;
}

void checkGLError (const char* func)
{

#if defined RELEASE
    return;
#endif

    if (!func) {
        return;
    }

    std::cout << func <<"() - \033[31;1m";

    switch (glGetError ()) {
        case GL_INVALID_ENUM :
            std::cout << "invalid enum" << std::endl; 
        break;

        case GL_INVALID_VALUE :
            std::cout << "invalid value" << std::endl; 
        break;

        case GL_INVALID_OPERATION :
            std::cout << "invalid operation" << std::endl; 
        break;

        case GL_INVALID_FRAMEBUFFER_OPERATION :
            std::cout << "invalid framebuffer operation" << std::endl; 
        break;

        case GL_OUT_OF_MEMORY :
            std::cout << "out of memory" << std::endl; 
        break;

        case GL_STACK_UNDERFLOW :
            std::cout << "stack underflow" << std::endl; 
        break;

        case GL_STACK_OVERFLOW :
            std::cout << "stack overflow" << std::endl; 
        break;

        default :
            std::cout << "\033[32;1mok" << std::endl; 
        break;
    }

    std::cout << "\033[0m";
}

void OpenGL::dumpGLInfo ()
{
    std::cout << "OpenGL-vendor:\n\t"
              << glGetString (GL_VENDOR)
              << "\n\n";
    std::cout << "OpenGL-renderer:\n\t"
              << glGetString (GL_RENDERER)
              << "\n\n";
    std::cout << "OpenGL-version:\n\t"
              << glGetString (GL_VERSION)
              << "\n\n";
    std::cout << "OpenGL-shading-language:\n\t"
              << glGetString (GL_SHADING_LANGUAGE_VERSION)
              << "\n\n";
}

GLuint OpenGL::loadShader (const char *src, GLenum type)
{
    GLuint shader = glCreateShader (type);
    if (shader)
    {
        GLint compiled;

        glShaderSource (shader, 1, &src, NULL);
        glCompileShader (shader);
        glGetShaderiv (shader, GL_COMPILE_STATUS, &compiled);
        if (!compiled)
        {
            GLchar log[1024];
            glGetShaderInfoLog (shader, sizeof log - 1, NULL, log);
            log[sizeof log - 1] = '\0';
            std::cout << "loadShader compile failed: " << log << std::endl;
            glDeleteShader (shader);
            shader = 0;
        }
    }

    return shader;
}

GLuint OpenGL::createShaderProgram (const char* vertSrc,
									const char* fragSrc,
									bool link)
{
    if (!vertSrc && !fragSrc)
        return 0;

    if (vertSrc) {
        _vShaderId = loadShader (vertSrc, GL_VERTEX_SHADER);
        assert (_vShaderId);
    }

    if (fragSrc) {
        _fShaderId = loadShader (fragSrc, GL_FRAGMENT_SHADER);
        assert (_fShaderId);        
    }

    _program = glCreateProgram ();
    assert (_program);
    if (vertSrc) {
        glAttachShader (_program, _vShaderId);
    }

    if (fragSrc) {
        glAttachShader (_program, _fShaderId);
    }

    if (!link) {
        return _program;
    }

    checkGLError (nullptr);
    glProgramParameteri(_program, GL_PROGRAM_BINARY_RETRIEVABLE_HINT, GL_TRUE);
    checkGLError ("glProgramParameteri");

    glLinkProgram (_program);

    GLint linked = 0;
    glGetProgramiv (_program, GL_LINK_STATUS, &linked);
    if (!linked)
    {
        GLchar log[1024];
        glGetProgramInfoLog (_program, sizeof log - 1, NULL, log);
        log[sizeof log - 1] = '\0';
        std::cout << "Link failed: " << log << std::endl;
        glDeleteProgram (_program);
        return 0;
    }

    return _program;
}

void OpenGL::linkShaderProgram (GLuint progId)
{
    glLinkProgram (progId);

    GLint linked = 0;
    glGetProgramiv (progId, GL_LINK_STATUS, &linked);
    if (!linked)
    {
        GLchar log[1024];
        glGetProgramInfoLog (progId, sizeof log - 1, NULL, log);
        log[sizeof log - 1] = '\0';
        std::cout << "Link failed: " << log << std::endl;
        glDeleteProgram (progId);
    }
}

void OpenGL::dumpProgramBinaryInfoToConsole (GLuint progId) const
{
    GLint binaryLength = 0;
    glGetProgramiv (progId, GL_PROGRAM_BINARY_LENGTH, &binaryLength);
    std::cout << "binary-length: " << binaryLength << '\n';
}