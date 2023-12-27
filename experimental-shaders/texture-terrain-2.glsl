// build/raymarcher-gl experimental-shaders/texture-terrain-2.glsl 1280 720 60 data/rock-4.jpg
#version 420
uniform vec3 iResolution;
uniform float iTime;
uniform vec3 iChannelResolution0;
uniform vec3 iChannelResolution1;
uniform vec3 iChannelResolution2;
uniform vec3 iChannelResolution3;
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

const int MAX_ITER = 48;
const float EPSILON = .0001;
const float STEP_BIAS = .65;

mat2 r2d (float deg) {
	float rad = radians (deg);
	float c = cos (rad);
	float s = sin (rad);
	return mat2 (c, s, -s, c);
}

// hash() & noise3d() are from an example by iq or shane... I can't remember 
float hash (float f)
{
	return fract (sin (f) * 45785.5453);
}

float noise3d (vec3 p)
{
    vec3 u = floor (p);
    vec3 v = fract (p);
    
    v = v * v * (3. - 2. * v);

    float n = u.x + u.y * 57. + u.z * 113.;
    float a = hash (n);
    float b = hash (n + 1.);
    float c = hash (n + 57.);
    float d = hash (n + 58.);

    float e = hash (n + 113.);
    float f = hash (n + 114.);
    float g = hash (n + 170.);
    float h = hash (n + 171.);

    float result = mix (mix (mix (a, b, v.x),
                             mix (c, d, v.x),
                             v.y),
                        mix (mix (e, f, v.x),
                             mix (g, h, v.x),
                             v.y),
                        v.z);

    return result;
}

// cheap-ass material-system with call-by-ref id & pout
float map (vec3 p, inout int id, inout vec3 pout) {
	vec3 tp = p;
	tp.z -= 6.*iTime;
	float lf = 2.*noise3d (.25*tp.xzy);
	float mf = 1.25*noise3d (.5*tp.xzy);
	float hf = .5*noise3d (2.*tp.xzy);
	float g = tp.y + 1.5 + lf + mf + hf;
	float w = tp.y + 3.5;
    pout = tp;
	vec3 bp = p;
	bp.x += 2.*cos (iTime)*sin(2.*iTime);
    bp.y += .75*cos (.75*iTime);
    float r = .5 + .2*(cos(5.*iTime - 7.*bp.z));
	float b = length (bp) - r;
	float d = min (b, min (g, w));
	if (d == g) id = 1;
	if (d == w) id = 2;
	if (d == b) id = 3;
	return d;
}

float march (vec3 ro, vec3 rd, float tmax, inout int id, inout int iters, inout vec3 pout) {
	float t = .0;
	float d = .0;
	for (int i = 0; i < MAX_ITER; ++i) {
		iters = i;
		vec3 p = ro + d*rd;
		t = map (p, id, pout);
		if (abs (t) < EPSILON*(1. + .125*t) || d > tmax) break;
		d += t*STEP_BIAS;
	}
	return d > tmax ? tmax : d;
}

vec3 norm (vec3 p) {
	int foo;
    vec3 bar;
	float d = map (p, foo, bar);
	vec2 e = vec2 (.001, .0);
	return normalize (vec3 (map (p + e.xyy, foo, bar),
                            map (p + e.yxy, foo, bar),
                            map (p + e.yyx, foo, bar)) - d);
}

float shadow (vec3 p, vec3 n, vec3 ldir, float ldist) {
	float tmax = 20.;
	int foo;
	int bar;
    vec3 bla;
	float d2w = march (p + .01*n, ldir, tmax, foo, bar, bla);
	return ldist < d2w ? 1. : .1;
}

float ao (vec3 p, vec3 n, float stepsize, int iters, float intensity) {
	float dist = .0;
	int foo;
    vec3 bla;
	float ao = .0;
	for (int i = 1; i <= iters; ++i) {
		dist = float (i)*stepsize;
		ao += max (.0, (dist - map(p + dist*n, foo, bla))/dist);
	}
	return 1. - ao*intensity;
}

// id 1: terrain
// id 2: water
// id 3: sphere/'spaceship'
vec3 shade (vec3 ro, vec3 rd, float d, vec3 p, vec3 n, int id, vec3 pout) {
	vec3 amb = vec3 (.05);
	vec3 lp = vec3 (-1., 2., 2.);
	vec3 ldir = normalize (lp - p);
	float li = 3.;
	vec3 lc = vec3 (.9, .8, .7);
	vec3 mat = vec3 (.2);
	if (id == 1) mat = texture (iChannel0, pout.xz).rgb+vec3 (.05, .15, .1);
	if (id == 2) mat = vec3 (.0, .0, .9);
	if (id == 3) mat = vec3 (.2, .1, .0);
	float ldist = distance (lp, p);
	float s = shadow (p, n, ldir, ldist);
	float att = 4. / (ldist*ldist);
	float diff = max (.0, dot (n, ldir));
	vec3 h = normalize (-rd + ldir);
	float shiny = 100.;
	if (id == 1) shiny = 10.;
	if (id == 2) shiny = 100.;
	if (id == 3) shiny = 50.;
	float sp = pow (max (.0, dot (n, h)), shiny);
	float ao = 1.;//ao (p, n, .02, 8, .2); // ao has no visual impact currently
	return amb + ao*att*s*(diff*lc*li*mat + sp*vec3 (1.));
}

vec3 background (vec2 uv) {
	float m = smoothstep (.0, 1.125, clamp (uv.y, .0, 1.));
	return 5.*mix (vec3 (.9, .85, .5), vec3 (.2, .1, .9), m);
}

vec3 cam (vec2 uv, vec3 ro, vec3 aim, float zoom) {
	vec3 f = normalize (aim - ro);
	vec3 wu = vec3 (.0, 1., .0);
	vec3 r = normalize (cross(wu, f));
	vec3 u = normalize (cross(f, r));
	vec3 c = ro + f*zoom;
	return normalize (c + r*uv.x + u*uv.y - ro);
}

void main ()
{
    vec2 uv = fragCoord.xy;
	vec2 uvRaw = uv;
	uv = uv*2. - 1.;
	uv.x *= iResolution.x/iResolution.y;
	uv *= 1. + .35*length(uv);

    float heightOffset = +.4*cos(iTime);
	vec3 ro = vec3 (1., 2. + heightOffset, 7.);
	vec3 aim = vec3 (.0, 1.5 + heightOffset, .0);
	float zoom = 2.;
	vec3 rd = cam (uv, ro, aim, zoom);
	rd.xy *= r2d (15.*cos (iTime));

	float tmax = 90.;
	int id = 0;
	int iters = 0;
    vec3 pout = vec3 (.0);
	float d = march (ro, rd, tmax, id, iters, pout);
	float d2 = d;
	vec3 col = vec3 (.0);
	float fog = 15. / (1. + d*d*.025);

	if (d == tmax) {
		uv *= r2d (-15.*cos (iTime));
		col = background (uv);
	} else {
		vec3 p = ro + d*rd;
		vec3 n = norm (p);
		col = shade (ro, rd, d, p, n, id, pout);
		if (id == 2 || id == 3) {
			ro = p + .01*n;
			rd = normalize (reflect (rd, n));
			d = march (ro, rd, tmax, id, iters, pout);
			p = ro + d*rd;
			n = norm (p);
			col += shade (ro, rd, d, p, n, id, pout);
		}
	}

	col *= fog; // fog
    col = mix (col, 1.5*vec3 (.95, .9, .8), pow (1. - 1. / d2, 130.)); // horizon-mist
	col = col / (1. + col); // tone-mapping
	col *= 1. - .5*length (uvRaw*2. - 1.); // vignette
	col *= mix (1., .5, cos (800.*uvRaw.y)); // raster-line
    col = pow (col, vec3 (1./2.2)); // gamma-correction

	fragColor = vec4(col, 1.);
}
