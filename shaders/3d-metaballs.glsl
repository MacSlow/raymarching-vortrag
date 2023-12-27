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

    const float EPSILON = .0001;
    const int MAX_ITER = 96;

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

	float scene (in vec3 p) {
	    vec3 pBottom = p;
	    vec3 pTop = p;

	    float r1 = .1 + .3 * (.5 + .5 * sin (2. * iTime));
	    float r2 = .1 + .25 * (.5 + .5 * sin (3. * iTime));
	    float r3 = .1 + .3 * (.5 + .5 * sin (4. * iTime));
	    float r4 = .1 + .25 * (.5 + .5 * sin (5. * iTime));

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

	    return min (metaBalls, min (hexBottom, hexTop));
	}

	float raymarch (in vec3 ro, in vec3 rd, out int iter) {
	    float t = .0;
	    float d = .0;
	    iter = 0;
	    for (int i = 0; i < MAX_ITER; ++i) {
	        iter++;
	        vec3 p = ro + d * rd;
	        t = scene (p);
	        if (abs (t) < EPSILON * (1. + .125*t)) break;
	        d += t*.7;
	    }

	    return d;
	}

	vec3 normal (in vec3 p, in float epsilon) {
		float d = scene (p);
	    vec3 e = vec3 (epsilon, .0, .0);
	    return normalize (vec3 (scene (p + e.xyy) - d,
	                            scene (p + e.yxy) - d,
	                            scene (p + e.yyx) - d));
	}

	float shadow (in vec3 p, in vec3 lPos)
    {
		float distanceToLight = distance (p, lPos);
		vec3 n = normal (p, EPSILON);
		int ignored = 0;
		float distanceToObject = raymarch (p + .01*n,
										   normalize (lPos - p),
										   ignored);
		bool isShadowed = distanceToObject < distanceToLight;
		return isShadowed ? .1 : 1.;
    }

	vec3 shade (in vec3 ro, in vec3 rd, in float d) {
		vec3 p = ro + d*rd;
	    vec3 amb = vec3 (.1);
		vec3 diffC = vec3 (1., .5, .3);
	    vec3 specC = vec3 (1., .95, .9);
		vec3 diffC2 = vec3 (.3, .5, 1.);
	    vec3 specC2 = vec3 (.9, .95, 1.);

	    vec3 n = normal (p, d*EPSILON);
	    vec3 lPos = ro + vec3 (.5, 1.0, -3.);
	    vec3 lPos2 = ro + vec3 (-1., 1.2, 2.);
	    vec3 lDir = lPos - p;
	    vec3 lDir2 = lPos2 - p;
	    vec3 lnDir = normalize (lDir);
	    vec3 lnDir2 = normalize (lDir2);
	    float sha = shadow (p, lPos);
	    float sha2 = shadow (p, lPos2);
		float lDist = distance (p, lPos);
		float lDist2 = distance (p, lPos2);
		float attenuation = 8. / (lDist*lDist);
		float attenuation2 = 8. / (lDist2*lDist2);

	    float diff = max (dot (n, lnDir), .0);
	    float diff2 = max (dot (n, lnDir2), .0);
		vec3 h = normalize (lDir - rd);
		vec3 h2 = normalize (lDir2 - rd);
	    float spec = pow (max (dot (h, n), .0), 20.);
	    float spec2 = pow (max (dot (h2, n), .0), 40.);

	    vec3 diffTerm = sha * attenuation * diff * diffC;
	    vec3 diffTerm2 = sha2 * attenuation2 * diff2 * diffC2;
	    vec3 specTerm = (sha > .1) ? attenuation * spec * specC : vec3 (.0);
	    vec3 specTerm2 = (sha2 > .1) ? attenuation2 * spec2 * specC2 : vec3 (.0);

		return amb + diffTerm + specTerm + diffTerm2 + specTerm2;
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
		vec2 uvRaw = uv;
	    uv = uv *2. - 1.;
	    uv.x *= iResolution.x / iResolution.y;

	    float angle = radians (300. + 55. * iTime);
	    float dist = 3. + cos (1.5*iTime);
	    vec3 ro = vec3 (dist * cos (angle), .0, dist * sin (angle));
	    vec3 aim = vec3 (.0);
	    float zoom = 2.;
	    vec3 rd = camera (uv, ro, aim, zoom);

	    int iter = 0;
	    float d = raymarch (ro, rd, iter);
		float fog = 1. / (1. + d*d*.05);
	    vec3 p = ro + d * rd;
	    
	    vec3 n = normal (p, d*EPSILON);
	    vec3 col = shade (ro, rd, d);

		vec3 refl = normalize (reflect (rd, n));
		float refd = raymarch (p + .01*n, refl, iter);
		vec3 refp = p + refd*refl;
		vec3 refc = shade (p, refl, refd);
		float fakeFresnel = pow (1. - max (dot (n, -rd), .0), 1.75);
		col += .35*fakeFresnel*fakeFresnel*refc;

		col *= fog;
	    col = mix (col, vec3 (.95, .85, .7), pow (1. - 1. / d, 17.));
	    col = col / (.75 + col);
	    col = .2 * col + .8 * sqrt (col);
		col *= .7 + .3 * pow (16. * uvRaw.x * uvRaw.y * (1. - uvRaw.x) * (1. - uvRaw.y), .3);

		fragColor = vec4 (col, 1.);
	}

