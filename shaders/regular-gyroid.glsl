//build/raymarcher-gl  shaders/regular-gyroid.glsl 640 360 30 data/noise-256x256.png
#version 420
uniform vec3 iResolution;
uniform sampler2D iChannel0;
uniform float iTime;
uniform float iFrameRate;
uniform int  iFrame;
in vec2 fragCoord;
out vec4 fragColor;

precision highp float;

const float PI = 3.14159265359;

mat2 r2d (float deg) {
	float rad = radians (deg);
	float c = cos (rad);
	float s = sin (rad);
	return mat2 (c,s,-s,c);
}

float gyroid (vec3 p, float size, float thickness, float scale) {
	float surfaceSide = dot (scale*sin (p), scale*cos (p.yzx));
	float d = abs (surfaceSide) - thickness;
	vec3 a = abs (p);
	return max (d, max (a.x, max (a.y, a.z)) - size);
}

float map (vec3 p, inout int id, inout vec3 pout) {
	float ground = p.y + 2.;

	vec3 pgyroid = p + vec3 (.0, .6, .0);;
	pgyroid.xz *= r2d (45.*iTime);
	float gyroid = gyroid (3.*pgyroid, 4., .075, .75);

	float d = min (ground, gyroid);

	if (d == ground) {id = 1; pout = p;}
	if (d == gyroid) {id = 2; pout = pgyroid;}

    return d;
}

float march (vec3 ro, vec3 rd, inout int id, inout vec3 pout)
{
	float t = .0;
	float d = .0;
	for (int i = 0; i< 96; ++i) {
		vec3 p = ro+d*rd;
		t = map (p, id, pout);
		if (abs (t) < .00001*(1. + .125*t)) break;
		d += t*.25;
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

float ao (vec3 p, vec3 n, float stepsize, float i) {
	float ao = .0;
	float dist = .0;
	int foo;
	vec3 bar;
	for (int a = 1; a <= 8; ++a) {
		dist = float (a)*stepsize;
		ao += max (.0, (dist - map (p+n*dist, foo, bar))/dist);
	}
	return 1. - ao*i;
}

vec3 shade (vec3 ro, vec3 rd, float d, vec3 n, vec3 lp, vec3 lc, float li, int id, vec3 pout) {
    vec3 p = ro + d*rd;
	float ld = distance (p, lp); 
	vec3 ldir = normalize (lp - p);
	float att = 5. / (ld*ld);
	vec3 mat = vec3 (.2);
	if (id == 1) {
		float f = texture (iChannel0, .5*pout.xz*vec2 (.3*cos(10.+pout.x+pout.z), 2.)).r;
		mat = mix (vec3 (.3, .2, .1),
				   vec3 (.6, .55, .3),
				   smoothstep(.0, .9, f));
	}
	if (id == 2) mat = mix (vec3 (.1), vec3 (.3), smoothstep (.1,.9,cos (60.*pout.y)));
	float s = sha (p, lp, n, ldir);
	float diff = max (.0, dot (n, ldir));
	vec3 h = normalize (-rd + ldir);
	float shiny = 100.;
	float sp = pow (max (.0, dot (n, h)), shiny);
	vec3 am = vec3 (.05);
	float ao = ao (p, n, .1, .1);
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
	uv *= 1. + .5*length (uv);
	
	float dist = 4.;
	float t = .2*iTime;
	vec3 ro = vec3 (dist*cos (t), 2., dist*sin(t));
	vec3 rd = cam (uv, ro, vec3 (.0, -.5, .0), 1.75);
	int id = 0;
	vec3 pout = vec3 (.0);
	float d = march (ro, rd, id, pout);
	float fog = 1. / (1. + d*d*.025);
	vec3 p = ro + d*rd;
	vec3 n = norm (p);
	vec3 col = shade (ro, rd, d, n, vec3 (2., 1.5, 3.), vec3 (.9, .85, .5), 6.,id, pout);
	col += shade (ro, rd, d, n, vec3 (.0, 4., 1.), vec3 (.2, .2, .9), 9.,id, pout);
	col += shade (ro, rd, d, n, vec3 (-3., 1., .5), vec3 (.9, .5, .3), 4.,id, pout);
	col += shade (ro, rd, d, n, vec3 (1., .5, -3.), vec3 (.5), 9.,id, pout);

	col *= fog;
	col = col / (1. + col);
	col *= 1. - .65*length(uvRaw*2.-1.);
	col = pow (col, vec3 (1./2.2));

    fragColor = vec4(col, 1.);
}
