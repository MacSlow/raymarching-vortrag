// build/raymarcher-gl shaders/texture-terrain.glsl 640 360 30 data/noise-256x256.png
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

const int MAX_ITER    = 64;
const float STEP_SIZE = .95;
const float EPSILON   = .001;
const float PI = 3.14159265359;

mat2 r2d (in float a) { float c = cos(a); float s = sin (a); return mat2 (vec2 (c, s), vec2 (-s, c));}

vec4 noise (in vec2 p)
{
	return texture (iChannel0, (p + .5) / iChannelResolution0.xy);
}

float scene (in vec3 p)
{
	p.z -= 4.*iTime;
	p.x += 2.*cos (.75*iTime);
    float hf = 2.*noise (24. * p.xz).x;
    float lf = .75*noise (1. * p.xz).x;
    float terrain = p.y - (hf+lf);

    return terrain;
}

vec3 normal (in vec3 p)
{
	float d = scene (p);
    vec2 e = vec2 (.001, .0);
	return normalize (vec3 (scene (p + e.xyy),
                            scene (p + e.yxy),
                            scene (p + e.yyx)) - d);
}

float raymarch (in vec3 ro, in vec3 rd, in float t, in float tmax)
{
    float d = t;
    for (int i = 0; i < 99; ++i) {
        d = scene (ro + t * rd);
        t += d*1.1;
        if (abs (t) < .0001*(1. + .125*t) || t >= tmax) break;
    }
	return t;
}

vec3 cam (vec2 uv, vec3 ro, vec3 aim, float zoom) {
	vec3 f = normalize (vec3 (aim - ro));
	vec3 wu = vec3 (.0, 1.,.0);
	vec3 r = normalize (cross (wu, f));
	vec3 u = normalize (cross (f, r));
	vec3 c = ro + f*zoom;
	return normalize (c + r*uv.x + u*uv.y - ro);
}

vec3 shade (vec3 ro, vec3 rd, float d, vec3 n)
{
	vec3 amb = vec3 (.06);
	vec3 p = ro + d*rd;
	vec3 lc = vec3 (.9, .8, .7);
	vec3 lp = p+vec3 (1.);
	vec3 ldir = normalize (lp - p);
	float ldist = distance (lp, p);
	float att = 7. / (ldist*ldist);
	float diff = max (.0, dot (n, ldir));
	vec3 mat = vec3 (.2, .1, .0);
	float li = 7.;
	return amb + att*diff*lc*li*mat;
}
void main ()
{
    // normalizing and aspect-correction
	vec2 uvRaw = fragCoord.xy;
	vec2 uv = uvRaw;
    uv = uv * 2. - 1.;
    uv.x *= iResolution.x / iResolution.y;
	uv *= 1. + .25*length(uv);

    // set up "camera", view origin (ro) and view direction (rd)
    vec3 ro = vec3 (.9, 4., 3.);
	vec3 rd = cam (uv, ro, vec3 (.0, 2., .0), 1.73);
	rd.xy *= r2d (.2*cos(iTime));

    float t = .1;
    float tmax = 30.0;
    float d = raymarch (ro, rd, t, tmax);
    vec3 c = vec3 (.0);
    vec3 n = normal (ro + d * rd);
	vec3 shade = shade (ro, rd, d, n);
    c = shade*vec3 (clamp (1. - sqrt (d / tmax), .0, 1.));

	c = c / (.75 + c*.35);
	c *= 1. - .35*length(uvRaw*2. - 1.);
	c *= mix (1., .25, cos (700.*uvRaw.y));
	c = pow (c, vec3 (1./2.2));

    fragColor = vec4 (c, 1.);
}

