#version 420

uniform vec3 iResolution;
uniform float iTime;
uniform float iTimeDelta;
uniform vec3 iChannelResolution0;
uniform vec3 iChannelResolution1;
uniform vec3 iChannelResolution2;
uniform vec3 iChannelResolution3;
uniform float iFrameRate;
uniform int  iFrame;
uniform vec4 iMouse;
uniform sampler2D iChannel0;
uniform sampler2D iChannel1;
uniform sampler2D iChannel2;
uniform sampler2D iChannel3;
uniform vec4 iDate;
in vec2 fragCoord;
out vec4 fragColor;

precision highp float;

mat2 r2d (in float degree)
{
	float rad = radians (degree);
	float c = cos (rad);
	float s = sin (rad);
	return mat2 (vec2 (c, s),vec2 (-s, c));
}

vec2 hash (in vec2 p)
{
	p = vec2 (dot (p, vec2 (127.1, 311.7)),
			  dot (p, vec2 (269.5, 183.3)));

	return -1. + 2.*fract (sin (p)*43758.5453123);
}

float noise (in vec2 p)
{
    const float K1 = .366025404;
    const float K2 = .211324865;

	vec2 i = floor (p + (p.x + p.y)*K1);
	
    vec2 a = p - i + (i.x + i.y)*K2;
    vec2 o = step (a.yx, a.xy);    
    vec2 b = a - o + K2;
	vec2 c = a - 1. + 2.*K2;

    vec3 h = max (.5 - vec3 (dot (a, a), dot (b, b), dot (c, c) ), .0);

	vec3 n = h*h*h*h*vec3 (dot (a, hash (i + .0)),
						   dot (b, hash (i + o)),
						   dot (c, hash (i + 1.)));

    return dot (n, vec3 (70.));
}

float fbm (in vec2 p)
{
	mat2 rot = r2d (27.5);
    float d = noise (p); p *= rot;
    d += .5*noise (p); p *= rot;
    d += .25*noise (p); p *= rot;
    d += .125*noise (p); p *= rot;
    d += .0625*noise (p);
	d /= (1. + .5 + .25 + .125 + .0625);
	return .5 + .5*d;
}

vec2 mapToScreen (in vec2 p, in float scale)
{
    vec2 res = p;
    res = res * 2. - 1.;
    res.x *= iResolution.x / iResolution.y;
    res *= scale;
    
    return res;
}

vec2 cart2polar (in vec2 cart)
{
    float r = length (cart);
    float phi = atan (cart.y, cart.x);
    return vec2 (r, phi); 
}

vec2 polar2cart (in vec2 polar)
{
    float x = polar.x*cos (polar.y);
    float y = polar.x*sin (polar.y);
    return vec2 (x, y); 
}

void main()
{
    vec2 uv = mapToScreen (fragCoord.xy, 3.);

	uv *= r2d (12.*iTime);
	float len = length (uv);
	float thickness = .25;
	float haze = 2.5;

	// distort UVs a bit
	uv = cart2polar (uv);
	uv.y += .2*(.5 + .5*sin(cos (uv.x)*len));
	uv = polar2cart (uv);

    float d1 = abs ((uv.x*haze)*thickness / (uv.x + fbm (uv + 1.25*iTime)));
    float d2 = abs ((uv.y*haze)*thickness / (uv.y + fbm (uv - 1.5*iTime)));
    float d3 = abs ((uv.x*uv.y*haze)*thickness / (uv.x*uv.y + fbm (uv - 2.*iTime)));
    vec3 col = vec3 (.0);
	float size = .075;
	col += d1*size*vec3 (.1, .8, 2.);
	col += d2*size*vec3 (2., .1, .8);
	col += d3*size*vec3 (.8, 2., .1);

    fragColor = vec4 (col, 1.);
}

