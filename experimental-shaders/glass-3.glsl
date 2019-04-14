//build/raymarcher-gl experimental-shaders/glass.glsl 640 360 30 data/noise-256x256.png
#version 420
uniform vec3 iResolution;
uniform sampler2D iChannel0;
uniform float iTime;
in vec2 fragCoord;
out vec4 fragColor;

precision highp float;

bool isInside = false;

mat2 r2d (float deg) {
	float rad = radians (deg);
	float c = cos (rad);
	float s = sin (rad);
	return mat2 (c,s,-s,c);
}

float smin (float d1, float d2, float k) {
	float h = clamp (.5 + .5*(d2 - d1)/k, .0, 1.);
	return mix (d2, d1, h) - h*k*(1. - h);
}

float sdBox (in vec3 p, in vec3 size, in float r)
{
	vec3 d = abs(p) - size;
	return min (max (d.x, max (d.y,d.z)), .0) + length (max (d, .0)) - r;
}

float map (vec3 p, inout int id, inout vec3 pout) {
	float ground = p.y + 1.;
	float wall = p.z + 1.;
	vec3 pbox = p + vec3 (.0, -.4, -.2);
	pbox.xz *= r2d (14.*iTime);
	pbox.yz *= r2d (26.*iTime);
	float r = .75 + .1*(.5+.5*cos(4.*iTime + 9.*pbox.y));
	float box = sdBox (pbox, vec3 (.6), .05);
	float box2 = sdBox (pbox, vec3 (.55), .06);
	box = max (box, -box2);
	p -= vec3 (2.5*cos (1.25*iTime), .75, .2);
	p.xz *= r2d (-60.*iTime);
	p.yz *= r2d (-90.*iTime);
	float ball = sdBox (p , vec3 (.4), .05);
	box = (isInside? -1. : 1.)*box; //smin (box, ball, .5);
	float d = min (ground, min (wall, box));
	if (d == ground) {id = 1; pout = p;}
	if (d == wall) {id = 2; pout = p;}
	if (d == box) {id = 3; pout = p;}
    return d;
}

float march (vec3 ro, vec3 rd, inout int id, inout vec3 pout)
{
	float t = .0;
	float d = .0;
	for (int i = 0; i< 48; ++i) {
		vec3 p = ro+d*rd;
		t = map (p, id, pout);
		if (abs (t) < .00001*(1. + .125*t)) break;
		d += t*.75;
	}
	return d;
}

vec3 norm (vec3 p){
	int foo;
	vec3 bar;
	float d = map (p, foo, bar);
	vec2 e = vec2 (.001, .0);
	return normalize (vec3 (map (p+e.xyy, foo, bar),
                            map (p+e.yxy, foo, bar),
                            map (p+e.yyx, foo, bar))-d);
}

float sha (vec3 p, vec3 lp, vec3 n, vec3 ldir) {
	float d2l = distance (lp, p);
	int foo;
	vec3 bar;
	float d2w = march (p+.01*n, ldir, foo, bar);
	return d2l < d2w ? 1. : .1;
}

float ao (vec3 p, vec3 n, float stepsize, int iter, float i){
	float ao = .0;
	float dist = .0;
	int foo;
	vec3 bar;
	for (int a = 1; a <= iter; ++a) {
		dist = float (a)*stepsize;
		ao += max (.0, (dist - map (p+n*dist, foo, bar))/dist);
	}
	return 1. - ao*i;
}

vec3 shade (vec3 ro,
			vec3 rd,
			float d,
			vec3 n,
			vec3 lp,
			vec3 lc,
			float li,
			int id,
			vec3 pout) {
    vec3 p = ro + d*rd;
	float ld = distance (p, lp); 
	vec3 ldir = normalize (lp - p);
	float att = 5. / (ld*ld);
	vec3 mat = vec3 (.2);
	if (id == 1) {
		mat = mix (vec3 (.0, .0, .0),
				   vec3 (.5, .5, .5),
                   smoothstep(.4, .6, cos (5.*p.x) * sin(5.*p.z+5.*iTime)));
	}
	if (id == 2) {
		mat = mix (vec3 (.0, .0, .0),
				   vec3 (.5, .5, .5),
				   smoothstep (.0, .9, sin (25.*p.y + 12.*iTime)));
	}
	if (id == 3) {
		mat = vec3 (.9, .5, .2);
	}
	float s = sha (p, lp, n, ldir);
	float diff = max (.0, dot (n, ldir));
	vec3 h = normalize (-rd + ldir);
	float shiny = 100.;
	float lf = .0;
	float hf = .0;
    float fac = 1.;
    if (id == 1) {
		lf = texture (iChannel0, .5*p.xz + vec2 (.0, .5*iTime)).r;
		hf = texture (iChannel0, 2.*p.xz + vec2 (.0, 2.*iTime)).r;
		fac = lf + hf;
	}
    if (id == 2) {
		lf = texture (iChannel0, .5*p.xy + vec2 (.0, .5*iTime)).r;
		hf = texture (iChannel0, 2.*p.xy + vec2 (.0, 2.*iTime)).r;
		fac = lf + hf;
	}
	shiny *= fac;
	float sp = pow (max (.0, dot (n, h)), shiny);
	vec3 am = vec3 (.05);
	float ao = ao (p, n, .1, 8, .1);
	ao *= (isInside ? 1.25 : 1.);
	
	return ao*att*s*(am + diff*lc*li*mat + sp*vec3 (1.));
}

vec3 cam (vec2 uv, vec3 ro, vec3 aim, float zoom) {
	vec3 f =normalize (aim - ro);
	vec3 wu = vec3 (.0, 1., .0);
	vec3 r = normalize (cross (wu, f));
	vec3 u = normalize (cross (f, r));
	vec3 c = ro + f*zoom;
	return normalize (c + r*uv.x+u*uv.y - ro);
}

void main ()
{
	vec2 uvRaw = fragCoord.xy;
	vec2 uv = uvRaw*2. - 1.;
	uv.x *= iResolution.x/iResolution.y;
	uv *= 1. + .25*length (uv);

	vec3 ro = vec3 (cos (iTime), 1. + .125*(.5+.5*cos(5.*iTime)), 2.5);
	vec3 rd = cam (uv, ro, vec3 (.0), 1.7);
	int id = 0;
	vec3 pout = vec3 (.0);
	isInside = false;
	float d = march (ro, rd, id, pout);
	vec3 p = ro + d*rd;
	vec3 n = norm (p);
	vec3 col = vec3 (.0);

	vec3  lp1 = vec3 (2., 1., 2.);
	vec3  lc1 = vec3 (.9, .8, .7);
	float li1 = 3.;
	vec3  lp2 = vec3 (.7, 3., .0);
	vec3  lc2 = vec3 (.2, .2, .9);
	float li2 = 6.;
	vec3  lp3 = vec3 (-2., 2., .5);
	vec3  lc3 = vec3 (.9, .3, .2);
	float li3 = 3.;

	float fac = 1.;
	if (id == 3) {
		fac = .025;
	}

	col = fac*shade (ro, rd, d, n, lp1, lc1, li1, id, pout);
	col += fac*shade (ro, rd, d, n, lp2, lc2, li2, id, pout);
	col += fac*shade (ro, rd, d, n, lp3, lc3, li3, id, pout);
	if (id == 3) {
		n = normalize (n + texture (iChannel0, 1.75*p.xy).r);
		ro = p - .05*n;
		float ior = .85;
		rd = normalize (refract (rd, n, ior));
		isInside = true;
		d = march (ro, rd, id, pout);
		p = ro + d*rd;
		n = norm (p);
		col += shade (ro, rd, d, n, lp1, lc1, li1, id, pout);
		col += shade (ro, rd, d, n, lp2, lc2, li2, id, pout);
		col += shade (ro, rd, d, n, lp3, lc3, li3, id, pout);

		ro = p - .01*n;
		rd = normalize (refract (rd, n, ior));
		isInside = false;
		d = march (ro, rd, id, pout);
		p = ro + d*rd;
		n = norm (p);
		col += shade (ro, rd, d, n, lp1, lc1, li1, id, pout);
		col += shade (ro, rd, d, n, lp2, lc2, li2, id, pout);
		col += shade (ro, rd, d, n, lp3, lc3, li3, id, pout);

		n = normalize (n + texture (iChannel0, 1.75*p.xy).r);
		ro = p - .05*n;
		rd = normalize (refract (rd, n, ior));
		isInside = true;
		d = march (ro, rd, id, pout);
		p = ro + d*rd;
		n = norm (p);
		col += shade (ro, rd, d, n, lp1, lc1, li1, id, pout);
		col += shade (ro, rd, d, n, lp2, lc2, li2, id, pout);
		col += shade (ro, rd, d, n, lp3, lc3, li3, id, pout);

		ro = p - .01*n;
		rd = normalize (refract (rd, n, ior));
		isInside = false;
		d = march (ro, rd, id, pout);
		p = ro + d*rd;
		n = norm (p);
		col += shade (ro, rd, d, n, lp1, lc1, li1, id, pout);
		col += shade (ro, rd, d, n, lp2, lc2, li2, id, pout);
		col += shade (ro, rd, d, n, lp3, lc3, li3, id, pout);
	}

	for (int reflectionBounce = 0;
		   reflectionBounce < 2;
		   ++reflectionBounce) {
		if (id == 3) {
			n = normalize (n + texture (iChannel0, 1.75*p.xy).r);
			ro = p + .01*n;
			rd = reflect (rd, n);
			d = march (ro, rd, id, pout);
			p = ro + d*rd;
			n = norm (p);
			vec3 rcol = shade (ro, rd, d, n, lp1, lc1, li1, id, pout);
			rcol += shade (ro, rd, d, n, lp2, lc2, li2, id, pout);
			rcol += shade (ro, rd, d, n, lp3, lc3, li3, id, pout);
			float fakeFresnel = 2.; //pow (max (dot (n, -rd), .0), 3.);
			col += fakeFresnel*rcol;
		}
	}

	col = col / (1 + col);
	col *= 1. - .5*length(uvRaw*2.-1.);
	col = pow (col, vec3 (1./2.2));

    fragColor = vec4(col, 1.);
}

