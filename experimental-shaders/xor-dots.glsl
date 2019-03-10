#version 420

uniform vec3 iResolution;
uniform float iTime;
uniform vec3 iChannelResolution0;
uniform vec3 iChannelResolution1;
uniform vec3 iChannelResolution2;
uniform vec3 iChannelResolution3;
uniform vec4 iMouse;
uniform sampler2D iChannel0;
uniform sampler2D iChannel1;
uniform sampler2D iChannel2;
uniform sampler2D iChannel3;
uniform vec4 iDate;
in vec2 fragCoord;
out vec4 fragColor;

precision highp float;

mat2 r2d (in float degrees)
{
	float rad = radians (degrees);
    float c = cos (rad);
    float s = sin (rad);
    return mat2 (vec2 (c, s), vec2(-s, c));
}

vec2 mapToScreen (in vec2 p, in float scale)
{
    vec2 res = p;
    res = res * 2. - 1.;
    res.x *= iResolution.x / iResolution.y;
    res *= scale;
    
    return res;
}

float xor (in float a, in float b)
{
	return a*(1. - b) + b*(1. - a);
}

void main()
{
    vec2 uv = mapToScreen (fragCoord.xy, 10. + 5.*sin (5.*iTime));
	uv *= r2d (15.*iTime);

	vec2 grid = fract (uv) - .5;
	vec2 cell = uv - grid;
    vec3 col = vec3 (.0);

	float c = .0;
	for (float y = -1.; y <= 1.; y++) {
		for (float x = -1.; x <= 1.; x++) {
			vec2 offset = vec2 (x, y);

			float spot = length (grid - offset);
			float distanceOverCells = length (cell + offset)*mix (.1, .4, .5 + .5*sin(iTime));

			float radius = mix (.1, 1.5, (.5 + .5*sin (distanceOverCells - 4.*iTime)));
			c = xor (c, smoothstep (radius, radius*.75, spot));
		}
	}
	vec3 a = vec3 (.1, .15, .2);
	vec3 b = vec3 (.9, .85, .8);
	col += vec3 (mix (a, b, mod (c, 2.)));

    fragColor = vec4 (col, 1.);
}
