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

float noise2D (in vec2 uv)
{
	return fract (sin (uv.x*173. + uv.y*6547.)*5647.);
}

float singleSmoothNoise2D (in vec2 uv, in float scale)
{
	vec2 lv = fract (uv*scale);
	lv = lv*lv*(3. - 2.*lv);
	vec2 id = floor (uv*scale);
	vec3 col = vec3 (noise2D(uv));

	float bl = noise2D (id);
	float br = noise2D (id + vec2 (1., .0));
	float b = mix (bl, br, lv.x);
	float tl = noise2D (id + vec2 (.0, 1.));
	float tr = noise2D (id + vec2 (1., 1.));
	float t = mix (tl, tr, lv.x);

	return mix (b, t, lv.y);
}

float smoothNoise2D (in vec2 uv)
{
	float color = singleSmoothNoise2D (uv, 4.);
	color += singleSmoothNoise2D (uv, 8.)*.5;
	color += singleSmoothNoise2D (uv, 16.)*.25;
	color += singleSmoothNoise2D (uv, 32.)*.125;
	color += singleSmoothNoise2D (uv, 62.)*.0625;

	return color / (1. + .5 + .25 + .125 + .0625);
}

mat2 r2d (in float degree)
{
	float rad = radians (degree);
	float c = cos (rad);
	float s = sin (rad);

	return mat2 (c, s, -s, c);
}

vec3 correctColor (in vec3 color, in vec2 uv)
{
	color = color / (1. + color);
	color = .1 * color + .9 * sqrt (color);
	color *= vec3 (.95, .9, .85);
	color *= .2 + .8*pow(16.*uv.x*uv.y*(1. - uv.x)*(1. - uv.y), .3);

	return color;
}

void main ()
{
    // normalizing and aspect-correction
	vec2 uv = fragCoord.xy;
	vec2 uvRaw = uv;
	float size = 2.;
    uv = uv * size - .5*size;
    uv.x *= iResolution.x/iResolution.y;

	// wraping texture-coords
	float t = 2.*iTime;
	uv *= r2d (t*16.);
	float tile_size = .2*size;
	float tile_x = uv.x/tile_size;
	float tile_y = uv.y/tile_size;
	float offset_x = uv.x + .95*sin (tile_x*.3 + tile_y*.15 + t);
	float offset_y = uv.y + 1.05*sin (tile_y*.3 + tile_x*.15 + t*.75);

	// moving 2D-noise
	uv *= r2d (-8.*iTime);
	uv += .4*iTime;
	float r = smoothNoise2D (vec2 (offset_x, offset_y)*r2d(1.1*sin(iTime*3.3)));
	float g = smoothNoise2D (vec2 (offset_x, offset_y)*r2d(2.2*sin(iTime*2.2)));
	float b = smoothNoise2D (vec2 (offset_x, offset_y)*r2d(3.1*sin(iTime*1.1)));
	vec3 color = vec3 (r, g, b);
	color = correctColor (color, uvRaw);

	fragColor = vec4 (color, 1.);
}

