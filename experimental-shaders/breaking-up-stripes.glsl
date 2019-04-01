#version 420
uniform vec3 iResolution;
uniform float iTime;
in vec2 fragCoord;
out vec4 fragColor;

precision highp float;

mat2 r2d (float deg) {
	float rad = radians (deg);
	float c = cos (rad);
	float s = sin (rad);
	return mat2 (c, s, -s, c);
}

float smin (float d1, float d2, float k)
{
	float h = clamp (.5 + .5*(d2 - d1)/k, .0, 1.);
	return mix (d2, d1, h) - k*h*(1. - h);
}

float sdSphere (vec3 p, float r) {
	return length (p) - r;
}

float ballMerge (float d, vec3 p, float r, float k) {
	float ball = sdSphere (p, r);
	return smin (d, ball, k);
}

float opRepeat (inout float p, float size) {
	float c = floor ((p + .5*size)/size);
	p = mod (p + .5*size, size) - .5*size;
	return c;
}

float map (vec3 p) {
	float ground = p.y + 1.5;
	float wall = p.z + 1.5;
	float size = 4.;
	float cell = opRepeat (p.x, 4.);
	p.xz *= r2d (45.*iTime + 80.*cell);
	p.xy *= r2d (57.*iTime + 50.*cell);
	float t = iTime;
	float r1 = .5 + .1*(.5+.5*cos (2.*t));
	float r2 = .3 + .1*(.5+.5*cos (3.*t));
	float r3 = .4 + .1*(.5+.5*cos (4.*t));
	float r4 = .2 + .1*(.5+.5*cos (2.*t));
	float r5 = .6 + .1*(.5+.5*cos (3.*t));
	float r6 = .3 + .1*(.5+.5*cos (4.*t));
	float k1 = .35 + .1*(.5+.5*cos (2.*t));
	float k2 = .5 + .1*(.5+.5*cos (3.*t));
	float k3 = .4 + .1*(.5+.5*cos (4.*t));
	float k4 = .6 + .1*(.5+.5*cos (2.*t));
	float k5 = .3 + .1*(.5+.5*cos (3.*t));
	vec3 c1 = vec3 (cos(2.*t), .1, sin(2.*t));
	vec3 c2 = vec3 (cos(3.*t), .2, sin(3.*t));
	vec3 c3 = vec3 (cos(4.*t), sin(2.*t), .3);
	vec3 c4 = vec3 (cos(2.*t), sin(4.*t), .5);
	vec3 c5 = vec3 (.2, cos (3.*t), sin(3.*t));
	vec3 c6 = vec3 (.5, cos (4.*t), sin(2.*t));
	float ball1 = sdSphere (p + c1, r1);
	ball1 = ballMerge (ball1, p + c2, r2, k1);
	ball1 = ballMerge (ball1, p + c3, r3, k2);
	ball1 = ballMerge (ball1, p + c4, r4, k3);
	ball1 = ballMerge (ball1, p + c5, r5, k4);
	ball1 = ballMerge (ball1, p + c6, r6, k5);
	float d = smin (ball1, smin (ground, wall, .7), .9);
	return d;
}

float march (vec3 ro, vec3 rd, inout int iter) {
	float t = .0;
	float d = .0;
	for (int i = 0; i < 48; ++i) {
		vec3 p = ro + d*rd;
		t = map (p);
		if (abs (t) < .001*(1. + .125*t)) break;
		d += t*.95;
		iter = i;
	}
	return d;
}

vec3 norm (vec3 p) {
	float d = map (p);
	vec2 e = vec2 (.001, .0);
	return normalize (vec3 (map (p+e.xyy),
                            map (p+e.yxy),
                            map (p+e.yyx)) - d);
}

float sha (vec3 p, vec3 lp, vec3 n, float ldist, vec3 ldir)
{
	int foo;
	float d2w = march (p+.01*n, ldir, foo);
	return ldist < d2w ? 1. : .1;
}

vec3 shade (vec3 ro, vec3 rd, float d, vec3 n, vec3 lp, vec3 lc, float li) {
	vec3 p = ro + d*rd;
	vec3 amb = vec3 (.05);
	vec3 ldir = normalize (lp - p);
	float ldist = distance (p, lp);
	float att = 7. / (ldist*ldist);
	float diff = max (.0, dot (n, ldir));
	vec3 mat = vec3 (.1, .2, 0);
	float m = smoothstep (.2, .3, .5+.5*cos(4.*(10.+p.z*p.y*p.y)));
	mat = mix (vec3 (1.), vec3 (.0), m);
	float s = sha (p, lp, n, ldist, ldir);
	vec3 h = normalize (-rd + ldir);
	float sp = pow (max (.0, dot (n, h)), 80.);
	return amb + s*att*(diff*lc*li*mat + sp*vec3 (1.));
}

vec3 cam (vec2 uv, vec3 ro, vec3 aim, float zoom) {
	vec3 f = normalize (aim - ro);
	vec3 wu = vec3 (.0, 1., .0);
	vec3 r = normalize (cross (wu, f));
	vec3 u = normalize (cross (f, r));
	vec3 c = ro + f*zoom;
	return normalize (c + r*uv.x + u*uv.y - ro);
}

void main ()
{
	vec2 uvRaw = fragCoord.xy;
	vec2 uv = uvRaw*2. - 1.;
	uv.x *= iResolution.x/iResolution.y;
	uv *= 1. + .75*length (uv);

    vec3 ro = vec3 (2., 2., 1.);
	vec3 aim = vec3 (.0);
	ro.x -= 7.*iTime;
	aim.x -= 7.*iTime;
	vec3 rd = cam (uv, ro, aim, 1.75);
	int iter = 0;
	float d = march (ro, rd, iter);
	float fog = 1 / (1. + d*d*.1);
	float glow = float (iter) / 48.;
	vec3 p = ro + d*rd;
	vec3 n = norm (p);
	vec3 lps[3] ={vec3 (.0, .0, 2.), vec3 (.0, 3., .0), vec3 (-3., 1., .5)};
	vec3 lcs[3] ={vec3 (.9, .8, .7), vec3 (.8, .7, .9), vec3 (.8, .9, .7)};
	vec3 col = shade (ro, rd, d, n, p+lps[0], lcs[0], 2. );
	col += shade (ro, rd, d, n, p+lps[1], lcs[1], 3.);
	col += shade (ro, rd, d, n, p+lps[2], lcs[2], 4.);
	col += pow (glow, 1.125)*vec3 (.9, .3, .1);

	ro = p+.01*n;
	rd = normalize (reflect (rd, n));
	d = march (ro,rd,  iter);
	vec3 rcol = shade (ro, rd, d, n, p+lps[0], lcs[0], 2.);
	rcol += shade (ro, rd, d, n, p+lps[1], lcs[1], 3.);
	rcol += shade (ro, rd, d, n, p+lps[2], lcs[2], 4.);
	col += .15*rcol;

	col *= fog;
	col = col / (1.25 + col*.5);	
	col *= 1. - .65*length (uvRaw*2. - 1.);
	col *= mix (1., .75, cos (500.*uvRaw.y));
	col = pow (col, vec3 (1./2.2));

	fragColor = vec4(col, 1.);
}

