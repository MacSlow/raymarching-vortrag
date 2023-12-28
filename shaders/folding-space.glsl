#version 420

layout (location = 0) out vec4 fragColor;

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
//out vec4 fragColor;

precision highp float;

mat2 r2d (float deg)
{
    float r = radians (deg);
    float c = cos (r);
    float s = sin (r);
    return mat2 (c, s, -s, c);
}

vec3 smin (vec3 d1, vec3 d2, float k)
{
    vec3 h = clamp (.5 + .5*(d2 - d1)/k, .0, 1.);
    return mix (d2, d1, h) - h*k*(1. - h);
}

float vary (float v, float s)
{
    return v*(.5 + .5*cos(s*iTime));
}

// ollj suggested map() variation, which lets you control smooth-minimum distance and
// cube edge-radius with the mouse
float map (vec3 p, out int id)
{
    float t = iTime*.5;
    float s = 3.;
    for(float i = .0; i < 5.; i += 1.) {
        p.xz *= r2d (degrees(t + i));
        p.xy *= r2d (degrees(-t*.4 - i));
        p = smin (p, -p, -.25-3.*iMouse.x/iResolution.x);
        p -= s; s *= .625;
    }
    vec3 size = vec3 (.4);
    float d = length (max (vec3 (.0), abs (p) - size))-iMouse.y/iResolution.y;
    id = 3;
    return d;
}

// my original version of map()
float map2(vec3 p, out int id)
{
	float t = iTime*.5;
	float s = 3.;
	for(float i = .0; i < 5.; i += 1.) {
		p.xz *= r2d (degrees(t + i));
		p.xy *= r2d (degrees(-t*.4 - i));
		p = smin (p, -p, -1.25);
		p -= s;
		s *= .625;
	}
	vec3 size = vec3 (.4);
    float d = length (max (vec3 (.0), abs (p) - size)) - .1;
    id = 3;
    return d;
}

float march (vec3 ro, vec3 rd, out int id, out int iter)
{
	float t = .0;
	float d = .0;
	for (int i = 0;i < 48; ++i) {
		iter = i;
		t = map(ro + d*rd, id);
		if (abs(t) < .01*(1. + .125*t)) break;
		d += t*.95;
	}
	return d;
}

vec3 normal (vec3 p)
{
    int ignored;
    vec2 e = vec2 (.001, .0);
    float d = map (p, ignored);
    return normalize (vec3 (map (p + e.xyy, ignored),
                            map (p + e.yxy, ignored),
                            map (p + e.yyx, ignored)) - d);
}

float shadow (vec3 p, vec3 ldir, vec3 n, float ldist)
{
    int ignored;
    int ignored2;
    float d2w = march (p + .01*n, ldir, ignored, ignored2);
    return ldist < d2w ? 1. : .25;
}

float ao (vec3 p, vec3 n, float d)
{
	int ignored;
	return clamp (map (p + d*n, ignored)/d, .0, 1.);
}

vec3 shade (vec3 rd, vec3 p, vec3 n, vec3 lp, vec3 lc, float li, int id)
{
    vec3 am = vec3(.1);
    float ldist = distance (lp, p);
    float att = 800./(ldist*ldist);
    vec3 ldir = normalize(lp - p);
    vec3 h = normalize(-rd + ldir);
    float s = shadow(p, ldir, n, ldist);
    vec3 mat = vec3(.3);
    float shiny = 80.;
    if (id == 3){ mat = vec3 (1.,.5,.25); shiny = 70.;}
    float sp = pow (max (.0, dot (n, h)), shiny);
    float diff = max (.0, dot (n, ldir));
    float ao = ao (p, n, 1.5);

    return att*s*(am + ao*diff*li*lc*mat + sp*vec3 (1.));
}

vec3 gradient (float value) {
	vec3 color = vec3 (.2);
	if (value > .0 && value < .2) {
		color = vec3 (.3, .4, .5);
	} else if (value >= .2 && value < .4) {
		color = vec3 (.35, .5, .7);
	} else if (value >= .4 && value < .7) {
		color = vec3 (.4, .6, .8);
	} else if (value >= .7 && value < .9) {
		color = vec3 (.5, .7, .85);
	} else if (value >= .9) {
		color = vec3 (.85, .9, .95);
	}
	return color;
}

vec3 shadeToon (vec3 p, vec3 n, vec3 lp, vec3 rd)
{
	vec3 ldir=normalize(lp-p);
	float diff=max(.0,dot(n,ldir));
	vec3 color = gradient (diff);
	vec3 outline = vec3 (.0);

    return color;
}

vec3 camera (vec2 uv, vec3 ro, vec3 aim, float z)
{
    vec3 f = normalize (aim - ro);
    vec3 wu = vec3 (.0, 1., .0);
    vec3 r = normalize (cross (wu, f));
    vec3 u = normalize (cross (f, r));
    vec3 c = ro + f*z;

    return normalize (c + r*uv.x + u*uv.y - ro);
}

void main()
{
    vec2 uv = fragCoord.xy/* /iResolution.xy*/ *2. - 1.;
    float aspect = iResolution.x/iResolution.y;
    uv.x *= aspect;

    vec3 ro = vec3 (4., 0.5, 33.);
    vec3 aim = vec3 (.0);
    float zoom = 2.25;
    vec3 rd = camera (uv, ro, aim, zoom);

    int id=0;
	int iter=0;
    float d=march(ro,rd,id,iter);
	float fog=1./(1.+d*d*.05);
    vec3 p=ro+d*rd;
    vec3 n=normal(p);

    // light one
    vec3 lp1=vec3(10.,20.,30.);
    lp1.xz*=r2d(vary(45.,2.));
    vec3 lc1=vec3(.9,.8,.7);
    float li1=24.;

    // light two
	vec3 lp2=vec3(-10.,5.,30.);
    lp2.yz*=r2d(vary(45.,2.));
    vec3 lc2=vec3(.7,.8,.9);
    float li2=20.;

    // light three
	vec3 lp3=vec3(-8.,-20.,30.);
    lp3.xy*=r2d(vary (45.,2.));
    vec3 lc3=vec3(.2,.3,.9);
    float li3=22.;

    vec3 c;
	if (uv.x < uv.y*aspect - .01) {
    	c = shade (rd, p, n, lp1, lc1, li1, id);
    	c += shade (rd, p, n, lp2, lc2, li2, id);
    	c += shade (rd, p, n, lp3, lc3, li3, id);
		c *= fog;
    	c = c/(1. + c);
		c *= 1. - .25*pow (length (uv), 2.);
    	c = pow (c, vec3 (1./2.2));
	} else if (uv.x >= uv.y*aspect - .01 && uv.x < uv.y*aspect + .01) {
		c = vec3 (1.);
	} else {
		float glow = float (iter)/48.;
		c = shadeToon (p, n, lp2, rd);
    }

    fragColor = vec4 (c, 1.);
}

