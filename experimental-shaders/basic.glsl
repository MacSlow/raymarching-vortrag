#version 420
uniform vec3 iResolution;
uniform float iTime;
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

void main ()
{
    // normalize and aspect-correct
    vec2 uv = fragCoord.xy;
	float c = .5 + .5*cos (iTime);
	float s = .5 + .5*sin (iTime);
    fragColor = vec4 (uv.x*c, uv.y*s, uv.x*uv.y, 1.);
}
