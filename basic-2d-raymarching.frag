#version 130

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

float circle (in vec2 p, in float r)
{
    return length (p) - r;
}

float rectangle (in vec2 p, in vec2 size, in float r)
{
    return length (max (abs (p) - size + vec2(r), .0)) - r;
}

vec2 opRot (in vec2 p, in float degrees)
{
	float rad = radians (degrees);
    float c = cos (rad);
    float s = sin (rad);
    return p * mat2 (vec2 (c, s), vec2(-s, c));
}

float opCombine (in float d1, in float d2, in float r)
{
    float h = clamp (.5 + .5 * (d2 - d1) / r, .0, 1.);
    return mix (d2, d1, h) - r * h * (1. - h);
}

vec2 mapToScreen (in vec2 p)
{
    vec2 res = p;
    res = res * 2. - 1.;
    res.x *= iResolution.x / iResolution.y;
    res *= 4.;
    
    return res;
}

void main()
{
    vec2 uv = mapToScreen (fragCoord.xy);
    vec2 mouse = mapToScreen (iMouse.xy/ iResolution.xy);
	mouse.x *= -1.;

    vec2 point1 = uv;
    vec2 point2 = uv + vec2 (mouse);

    float rectangle = rectangle (opRot (point1, 45. * iTime), vec2 (1.25, 2.), .25);
    float circle = circle (point2, 1.25);
    float d = opCombine (rectangle, circle, .75);

    float f = fract (d);

    vec3 col;
    if (d > .0)
        col = vec3 (1. - smoothstep (.025, .0125, f));
    else
        col = vec3 (1., .0, .0);

    fragColor = vec4(col, 1.);
}

