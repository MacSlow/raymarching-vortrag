//build/raymarcher-gl experimental-shaders/beam-bending.glsl 640 360 30 data/noise-256x256.png
#version 420
uniform vec3 iResolution;
uniform sampler2D iChannel0;
uniform float iTime;
in vec2 fragCoord;
out vec4 fragColor;

precision highp float;

mat2 r2d (float deg) {
	float rad = radians (deg);
	float c = cos (rad);
	float s = sin (rad);
	return mat2 (c,s,-s,c);
}

float texNoise (vec2 p) {
	return texture (iChannel0, p).r;
}

float smin (float d1, float d2, float k) {
	float h = clamp (.5 + .5*(d2 - d1)/k, .0, 1.);
	return mix (d2, d1, h) - h*k*(1. - h);
}

float map (vec3 p, inout int id, inout vec3 pout) {
	float size = 3.;
	float offset = cos (.2*iTime);
	float offset2 = cos (iTime);
	float ground = p.y + size + offset2 + .75*texNoise (offset+.05*p.xz+.1*iTime);
	ground = min (ground, -p.y + size + offset2 + .75*texNoise (offset-.1*iTime+.075*p.xz + 1.));
	float wall = p.z + 3.*size;
	wall = min (wall, -p.z + 3.*size);
	wall = min (wall, p.x + 2.*size);
	wall = min (wall, -p.x + 2.*size);
	vec3 pbar = p;
	pbar.xz *= r2d (25.*cos (pbar.y + sin(3.*iTime)));
	pbar.xy *= r2d (9.*cos (pbar.y + sin(4.*iTime)));
	pbar.yz *= r2d (9.*sin (pbar.y + sin(2.*iTime)));

	pbar.x += .2*cos (pbar.y + 2.*iTime);
	pbar.z += -(.3*(cos(2.*iTime)))*sin (2.*pbar.y + 2.*iTime);
	float thickness = .5-.2*offset2*cos (2.*pbar.y+3.*iTime);
	vec3 s = vec3 (thickness, 3.5, thickness);
	float bar = length (max (vec3 (.0), abs (pbar) - s)) - .1;
	float d = min (wall, smin (ground, bar, 1.5));
	id = 1;
    pout = pbar;
	if (d == wall) {id = 2; pout = p;}
    return d;
}

float march (vec3 ro, vec3 rd, inout int id, inout vec3 pout)
{
	float t = .0;
	float d = .0;
	for (int i = 0; i< 64; ++i) {
		vec3 p = ro+d*rd;
		t = map (p, id, pout);
		if (abs (t) < .000000001*(1. + .125*t)) break;
		d += t*.5;
	}
	return d;
}

vec3 norm (vec3 p){
	int foo;
	vec3 bar;
	float d = map (p, foo, bar);
	vec2 e = vec2 (.01, .0);
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

float ao (vec3 p, vec3 n, float stepsize, float i){
	float ao = .0;
	float dist = .0;
	int foo;
	vec3 bar;
	for (int a = 1; a <= 6; ++a) {
		dist = float (a)*stepsize;
		ao += max (.0, (dist - map (p+n*dist, foo, bar))/dist);
	}
	return 1. - ao*i;
}

vec3 shade (vec3 ro, vec3 rd, float d, vec3 n, vec3 lp, vec3 lc, float li, int id, vec3 pout) {
    vec3 p = ro + d*rd;
	float ld = distance (p, lp); 
	vec3 ldir = normalize (lp - p);
	float att = 15. / (ld*ld);
	vec3 mat = vec3 (1., .0, .0);
	if (id == 1) mat = vec3 (.0);
	if (id == 2) mat = mix (vec3 (.0), vec3 (1.), smoothstep (.0, .5, sin (3.*p.y+5.*iTime)));
	float s = sha (p, lp, n, ldir);
	float diff = max (.0, dot (n, ldir));
	vec3 h = normalize (-rd + ldir);
	float shiny = 40.;
	float sp = pow (max (.0, dot (n, h)), shiny);
	vec3 am = vec3 (.05);
	float ao = ao (p, n, .2, .05);
	return am + ao*att*s*(diff*lc*li*mat + sp*vec3 (1.));
}

vec3 cam (vec2 uv,vec3 ro,vec3 aim,float z){
vec3 f=normalize(aim-ro);
vec3 wu=vec3(.0,1.,.0);
vec3 r=normalize(cross(wu,f));
vec3 u=normalize(cross(f,r));
vec3 c=ro+f*z;
return normalize(c+r*uv.x+u*uv.y-ro);
}

void main ()
{
	vec2 uvRaw = fragCoord.xy;
	vec2 uv = uvRaw*2. - 1.;
	uv.x *= iResolution.x/iResolution.y;
	uv *= 1. + .25*length (uv);

	float dist = 3.;
	vec3 ro = vec3 (dist*cos (iTime), .0, dist*sin(iTime));
	vec3 rd = cam (uv, ro, vec3 (.0), 1.25);
	int id = 0;
	vec3 pout = vec3 (.0);
	float d = march (ro, rd, id, pout);
	vec3 p = ro + d*rd;
	vec3 n = norm (p);
	vec3 c = shade (ro, rd, d, n, vec3 (.0, .0, 2.), vec3 (.9, .85, .3), 2.,id, pout);
	c += shade (ro, rd, d, n, vec3 (2., 2., -2.), vec3 (.5, .5, .9), 2.,id, pout);

	if (id == 1) {
		ro = p + .01*n;
		rd = normalize (reflect (rd, n));
		d = march (ro, rd, id, pout);
		p = ro + d*rd;
		n = norm(p);
		vec3 rc = shade (ro, rd, d, n, vec3 (.0, .0, 2.), vec3 (.9, .85, .3), 2.,id, pout);
		rc += shade (ro, rd, d, n, vec3 (2., 2., -2.), vec3 (.3, .3, .9), 2.,id, pout);
		c += .3*rc;
	}
	c=c/(1.25+c*.5);
	c*=1.-.65*length(uvRaw*2.-1.);
	c*=mix(1.,.75,cos(500.*uvRaw.y));
	c=pow(c,vec3(1./2.2));

    fragColor = vec4(c,1.);
}

