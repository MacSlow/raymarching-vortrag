// build/raymarcher-gl shaders/90s-rubber-cube.glsl 512 512 60 data/noise-256x256.png
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

const int MAX_ITER    = 32;
const float STEP_SIZE = 1.;
const float EPSILON   = .001;

//#define t iTime

mat2 r2d (float d) {
	float a = radians(d);
	float c = cos(a);
	float s = sin (a);
	return mat2 (vec2 (c, s), vec2 (-s, c));
}

float map (vec3 p, inout int id, inout vec3 pout, float t)
{
	// rotate the cube
	p.xz *= r2d (135.*t);
	p.zy *= r2d (90.*t);
	p.xy *= r2d (-73.*t);

	// twist the cube
	p.xy *= r2d (35.*cos(p.z + 2.*t));
	p.yz *= r2d (60.*sin(p.x + 3.*t));
	p.zx *= r2d (40.*sin(p.y + 3.*t));

    // build cube from infinite planes, so it's
    // easier to get the UVs per plane later
	float size = .75;
	float p1 = p.y - size;
	float p2 = -p.y - size;
	float p3 = p.z - size;
	float p4 = -p.z - size;
	float p5 = p.x - size;
	float p6 = -p.x - size;

	float d = max (p1, p2);
	d = max (d, p3);
	d = max (d, p4);
	d = max (d, p5);
	d = max (d, p6);

    // material/UV id
	if (d == p1) id = 1;
	if (d == p2) id = 2;
	if (d == p3) id = 3;
	if (d == p4) id = 4;
	if (d == p5) id = 5;
	if (d == p6) id = 6;

    // use this when trying iteration-glow
    //d = length (max (vec3 (.0), abs (p) - size));

    // scale UVs
	pout = 10.*p;

	return d;
}

float march (vec3 ro, vec3 rd, inout int id, inout vec3 pout, inout int iter, float tm)
{
	float t = .0;
	float d = .0;

    for (int i = 0; i < MAX_ITER; i++)
    {
        iter = i;
        t = map (ro+d*rd, id, pout, tm);
        if (abs (t) < EPSILON*(1. + .125*t)) break;
        d += t*STEP_SIZE;
    }

    return d;
}

vec3 background (vec2 uv, float tm)
{
	float d = fract (5.*length (uv) - 3.*tm);
	float m = smoothstep (.0, 1., d);
    return mix (vec3 (.0125), vec3 (.3, .0125, .0), 1. - m);
}

vec3 shade (int id, vec3 pout, float tm)
{
    vec3 color = vec3 (.0);
	float dark = .1;
	float light = .9;

	if (id == 1) {
		pout.xz *= r2d (90.*tm);
		float lf = 4.*texture (iChannel0, .25*pout.xz).r;
		float hf = .25*texture (iChannel0, 4.*pout.xz).r;
		float d = cos (2.*pout.z - 6.*tm + sin(2.*(pout.x+lf+hf)));
		float m = smoothstep (.3, .7, d);
		color = mix (vec3 (dark), vec3 (light), m);
	}
	if (id == 2) {
        float t = 2.*tm;
		pout.xz *= r2d (-135.*t);
		pout.xz *= 20.;
        float d = cos(pout.x*.12 + t)+
                  cos(pout.x*.13 - t)+
                  cos(pout.z*.1 + t)+
            	  cos(pout.z*.105 - t);
	    float m = smoothstep (.0, 1., d);

		color = mix (vec3 (dark), vec3 (light), m);
	}
	if (id == 3) {
		float lf = 2.*texture (iChannel0, .125*pout.xy).r;
		float hf = .125*texture (iChannel0, 8.*pout.xy).r;
		float d = cos (5.*(pout.x+lf+hf)-5.*(pout.y-lf+hf)+15.*tm);
		float m = smoothstep (.5, .55, d);
		color = mix (vec3 (dark), vec3 (light), m);
	}
	if (id == 4) {
		pout.xy *= 1. + 1.*(.5 + .5*cos(4.*iTime));
		pout.xy *= r2d (45.*tm);
		float lf = texture (iChannel0, .25*pout.xy-.6*tm).r;
		float hf = .25*texture (iChannel0, 2.*pout.xy+.4*tm).r;
		float d = cos (pout.x+lf+hf + sin(lf+hf*pout.y));
		float m = smoothstep (.0, 1., d);
		color = mix (vec3 (dark), vec3 (light), m);
	}
	if (id == 5) {
		float d = fract (length (.5*pout.zy)+2.*tm);
		float m = smoothstep (.45, .55, d);
		color = mix (vec3 (dark), vec3 (light), d);
	}
	if (id == 6) {
		vec2 off1 = 5.*vec2 (cos (1.6*iTime), sin (2.4*tm));
		vec2 off2 = 6.*vec2 (cos (2.1*iTime), sin (1.5*tm));
		float d1 = fract (length (pout.yz + off1) - .3);
		float d2 = fract (length (pout.yz + off2) - .3);
		float m1 = smoothstep (.5, .52, d1);
		float m2 = smoothstep (.5, .52, d2);
		float m = m1*(1. - m2) + m2*(1. - m1);
		color = mix (vec3 (dark), vec3 (light), m);
	}

    return color;
}

vec3 camera (in vec2 uv, in vec3 ro, in vec3 aim, in float zoom)
{
    vec3 f = normalize (vec3 (aim - ro));
    vec3 wu = vec3 (.0, 1., .0);
    vec3 r = normalize (cross (wu, f));
    vec3 u = normalize (cross (f, r));
    vec3 c = ro + f*zoom;

    return normalize (c + uv.x*r + uv.y*u - ro);
}

void main ()
{
	vec2 uvRaw = fragCoord.xy;
	vec2 uv = uvRaw*2. - 1.;
    uv.x *= iResolution.x/iResolution.y;

    vec3 ro = vec3 (1., 1.5, 2.);
    vec3 aim = vec3 (.0, .5*cos(4.*iTime), .0);
    float zoom = 1.5+.2*cos(3.*iTime);
    vec3 rd = camera (uv, ro, aim, zoom);

	int id = 0;
	vec3 pout;
    int iter = 0;
	
    float d;
    float fog;
    vec3 bg;
    vec3 fg;
	vec3 c = vec3 (.0);

	const float numSamples = 8.;
	for (float samples = .0; samples < numSamples; ++samples) {
		float t = iTime - samples/numSamples/60.;
    	d = march (ro, rd, id, pout, iter, t);
    	fog = abs (1./(2. - exp(d*d*.35)));
    	bg = background (uv, t);
    	fg = shade (id, pout, t);
		c += (d < 3.25) ? fg*fog : bg;
	}
	c /= numSamples;

    c = c / (1. + c);
	c *= 1. - .75*length (uvRaw*2.-1.);
	c *= mix (1., .5, cos(700.*uv.y));
	c *= mix (1., .5, cos(700.*uv.x));
    c = pow (c, vec3 (1./2.2));

	fragColor = vec4(c, 1.);
}
