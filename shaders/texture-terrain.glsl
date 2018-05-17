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

const int MAX_ITER    = 64;
const float STEP_SIZE = .95;
const float EPSILON   = .001;
const float PI = 3.14159265359;

float saturate (in float v) { return clamp (v, .0, 1.); }
mat2 r2d (in float a) { float c = cos(a); float s = sin (a); return mat2 (vec2 (c, s), vec2 (-s, c));}

vec4 noise (in vec2 p)
{
	return texture (iChannel0, (p + .5) / iChannelResolution0.xy);
}

float scene (in vec3 p)
{
    float h = noise (32. * p.xz).x * 2.;
    float terrain = p.y - h;

    return terrain;
}

vec3 normal (in vec3 p)
{
    vec2 e = vec2 (.01, .0);
	return normalize (vec3 (scene (p + e.xyy),
                            scene (p + e.yxy),
                            scene (p + e.yyx)) - scene (p));
}

float raymarch (in vec3 ro, in vec3 rd, in float t, in float tmax)
{
    float d = t;
    for (int i = 0; i < 99; ++i) {
        d = scene (ro + t * rd);
        t += d;
        if (d < .01 * t || t >= tmax) break;
    }
	return t;
}

void main ()
{
    // normalizing and aspect-correction
	vec2 uvRaw = fragCoord.xy;
	vec2 uv = uvRaw;
    uv = uv * 2. - 1.;
    uv.x *= iResolution.x / iResolution.y;

    // set up "camera", view origin (ro) and view direction (rd)
    vec3 ro = vec3 (.9*cos (iTime), 2.5, .9*sin(iTime) + 1. - iTime);
	vec3 rd = normalize (vec3 (uv, -1.));
	rd.xy *= r2d (.2*cos(iTime));

    float t = .1;
    float tmax = 20.0;
    float d = raymarch (ro, rd, t, tmax);
    vec3 c = vec3 (.0);
    vec3 n = normal (ro + d * rd);
    //c = vec3 (n);
    c = vec3 (sqrt (d / tmax));

    fragColor = vec4 (c, 1.);
}

