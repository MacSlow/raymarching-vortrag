#version 130

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

	float fbm (in vec2 p) {
		float v = 1.;//texture (iChannel0, p).r;
	    return v;
	}

	mat2 r2d (in float a) {
		float c = cos (radians (a));
	    float s = sin (radians (a));
	    return mat2 (vec2(c, s), vec2(-s, c));
	}

	vec2 opRepeat2 (inout vec2 p, in vec2 size) {
	    vec2 hsize = .5 * size;
	    vec2 cell = floor ((p + hsize) / size);
	    p = mod (p + hsize, size) - hsize;
	    return cell;
	}

	float opCombine (in float d1, in float d2, in float r)
	{
	    float h = clamp (.5 + .5 * (d2 - d1) / r, .0, 1.);
	    return mix (d2, d1, h) - r * h * (1. - h);
	}

	float sdSphere (in vec3 p, in float r) {
	    return length (p) - r;
	}

	float sdHexPrism (in vec3 p, in vec2 h) {
	    vec3 q = abs (p);
	    return max (q.z - h.y, max ((q.x * .866025 + q.y * .5), q.y) - h.x);
	}

	float sdThing (in vec3 p) {
	    p.xz *= r2d(20.*iTime);
	    p.yz *= r2d(-40.*iTime);
	    float r = 1.25 + .75 * (.5 + .5 * cos (p.y * 7.))*
	                     .75 * (.5 + .5 * cos (p.x * 7.))*
	                     .75 * (.5 + .5 * cos (p.z * 7.));
		float d = length (p) - r;
	    return d;
	}

	float map (in vec3 p) {
	    vec3 pBottom = p;
	    vec3 pTop = p;

	    float r1 = .1 + .3 * (.5 + .5 * sin (2. * iTime));
	    float r2 = .15 + .2 * (.5 + .5 * sin (3. * iTime));
	    float r3 = .2 + .2 * (.5 + .5 * sin (4. * iTime));
	    float r4 = .25 + .1 * (.5 + .5 * sin (5. * iTime));

	    float t = 2. * iTime;
	    vec3 offset1 = vec3 (-.1*cos(t), .1, -.2*sin(t));
	    vec3 offset2 = vec3 (.2, .2*cos(t), .3*sin(t));
	    vec3 offset3 = vec3 (-.2*cos(t), -.2*sin(t), .3);
	    vec3 offset4 = vec3 (.1, -.4*cos(t), .4*sin(t));
	    vec3 offset5 = vec3 (.4*cos(t), -.2, .3*sin(t));
	    vec3 offset6 = vec3 (-.2*cos(t), -.4, -.4*sin(t));
	    vec3 offset7 = vec3 (.3*sin(t), -.6*cos(t), .6);
	    vec3 offset8 = vec3 (-.3, .5*sin(t), -.4*cos(t));

	    float ball1 = sdSphere (p + offset1, r4);
	    float ball2 = sdSphere (p + offset2, r2);
		float metaBalls = opCombine (ball1, ball2, r1);

	    ball1 = sdSphere (p + offset3, r1);
	    ball2 = sdSphere (p + offset4, r3);
		metaBalls = opCombine (metaBalls, opCombine (ball1, ball2, .2), r2);

	    ball1 = sdSphere (p + offset5, r3);
	    ball2 = sdSphere (p + offset6, r2);
		metaBalls = opCombine (metaBalls, opCombine (ball1, ball2, .2), r3);

	    ball1 = sdSphere (p + offset7, r3);
	    ball2 = sdSphere (p + offset8, r4);
		metaBalls = opCombine (metaBalls, opCombine (ball1, ball2, .2), r4);

	    pBottom.yz *= r2d(90.);
	    vec2 cellBottom = opRepeat2 (pBottom.yx, vec2 (.75));

	    pTop.yz *= r2d(270.);
	    vec2 cellTop = opRepeat2 (pTop.yx, vec2 (.75));

	    float hexBottom = sdHexPrism (pBottom + vec3 (.0, .0, -3.), vec2 (.25, .75 + .2 * sin(cellBottom.y)*cos(cellBottom.x)));
	    float hexTop = sdHexPrism (pTop + vec3 (.0, .0, -3.), vec2 (.25, .75 + .2 * sin(cellTop.y)*cos(cellTop.x)));

	    //float thing = sdThing (p-vec3 (.0, .0, 6.));

	    return min (metaBalls, min (hexBottom, hexTop));
	}

	float march (in vec3 ro, in vec3 rd, out int iter) {
	    float t = .0;
	    float d = .0;
	    iter = 0;
	    for (int i = 0; i < 64; ++i) {
	        iter++;
	        vec3 p = ro + d * rd;
	        t = map (p);
	        if (t < .0001) break;
	        d += t*.975;
	    }

	    return d;
	}

	vec3 normal (in vec3 p) {
		float d = map (p);
	    vec3 e = vec3 (.001, .0, .0);
	    return normalize (vec3 (map (p + e.xyy) - d,
	                            map (p + e.yxy) - d,
	                            map (p + e.yyx) - d));
	}

	vec3 shade (in vec3 ro, in vec3 p) {
	    vec3 amb = vec3 (.01);
		vec3 diffC = vec3 (1., .0, .0);
	    vec3 specC = vec3 (.0);

	    vec3 n = normal (p);
	    vec3 lPos = ro + vec3 (.5, 1.0, -3.);
	    vec3 lDir = lPos - p;
	    vec3 lnDir = normalize (lDir);
	    float sha = 1.;

	    float diff = max (dot (n, lnDir), .0);
	    float spec = .0;
	    
		return amb + sha * (diff * diffC + spec * specC);
	}

	vec3 camera (in vec2 uv, in vec3 ro, in vec3 aim, in float zoom)
	{
	    vec3 camForward = normalize (vec3 (aim - ro));
	    vec3 worldUp = vec3 (.0, 1., .0);
	    vec3 camRight = normalize (cross (worldUp, camForward));
	    vec3 camUp = normalize (cross (camForward, camRight));
	    vec3 camCenter = ro + camForward * zoom;
	    
	    return normalize (camCenter + uv.x * camRight + uv.y * camUp - ro);
	}

	void main () {
		vec2 uv = fragCoord.xy;
	    uv = uv *2. - 1.;
	    uv.x *= iResolution.x / iResolution.y;

	    // set up "camera", view origin (ro) and view direction (rd)
	    float angle = radians (300. + 55. * iTime);
	    float dist = 3. + cos (1.5*iTime);
	    vec3 ro = vec3 (dist * cos (angle), .0, dist * sin (angle));
	    vec3 aim = vec3 (.0);
	    float zoom = 2.;
	    vec3 rd = camera (uv, ro, aim, zoom);

	    int iter = 0;
	    float d = march (ro, rd, iter);
	    vec3 p = ro + d * rd;
	    
	    vec3 n = normal (p);
	    vec3 col = shade (ro, p);
	    col = mix (col, vec3 (.95, .85, .7), pow (1. - 1. / d, 10.));

	    //col = col / (1. + col);
	    col = sqrt (col);

		fragColor = vec4 (col, 1.);
	}

