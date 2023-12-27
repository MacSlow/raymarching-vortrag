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

const int MAX_ITER    = 128;
const float STEP_SIZE = .95;
const float EPSILON   = .0002;
const vec4 red     = vec4 (1.0, 0.0, 0.0, 1.0);
const vec4 green   = vec4 (0.0, 1.0, 0.0, 1.0);
const vec4 blue    = vec4 (0.0, 0.0, 1.0, 1.0);

	mat3 rotX (in float a) {float c = cos(a); float s = sin (a); return mat3 (vec3 (1., .0, .0), vec3 (.0, c, s), vec3 (.0, -s, c));}
	mat3 rotY (in float a) {float c = cos(a); float s = sin (a); return mat3 (vec3 (c, .0, s), vec3 (.0, 1., .0), vec3 (-s, .0, c));}
	mat3 rotZ (in float a) {float c = cos(a); float s = sin (a); return mat3 (vec3 (c, s, .0), vec3 (-s, c, .0), vec3 (.0, .0, 1.));}
	mat2 rot2d (in float a) {float c = cos(a); float s = sin (a); return mat2 (vec2 (c, s), vec2 (-s, c));}

vec4 gradient (float v) {
    float steps = 2.;
    float step = 1. / steps;
    vec4 col = green;

    if (v >= .0 && v < step) {
        col = mix (green, blue, v * steps);
    } else if (v >= step && v < 2.0 * step) {
        col = mix (blue, red, (v - step) * steps);
    }
	    
    return col;
}

	vec3 opRepeat (in vec3 p, in vec3 size) {return mod (p, 2. * size) - size;}
	float sdTorus (in vec3 p, in vec2 t) { vec2 q = vec2 (length (p.xz) - t.x, p.y); return length (q) - t.y; }

	float sdTruchet (in vec3 p, in vec3 t)
	{
	    float offset = t.z;
	    vec3 p1 = vec3 (p - vec3 (offset, offset, .0)) * rotX (radians (90.));
	    vec3 p2 = vec3 (p - vec3 (.0, -offset, offset)) * rotZ (radians (90.));
	    vec3 p3 = vec3 (p - vec3 (-offset, .0, -offset)) * rotY (radians (90.));
	    
	    float t1 = sdTorus (p1, t.xy);
	    float t2 = sdTorus (p2, t.xy);
	    float t3 = sdTorus (p3, t.xy);

	    return min (t1, min (t2, t3));
	}

	float scene (in vec3 p)
	{
	    vec2 mouse = iMouse.xy;
	    if (iMouse.xy == vec2(.0)) mouse.xy = vec2 (212., 192.);
	    mat3 rot = rotX (radians (180. + mouse.y / iResolution.y * 360.)) * rotY (radians (-180. + mouse.x / iResolution.x * 360.));
		p *= rot;
	
		p.x -= iTime * .25;

	    vec3 cellParam = vec3 (.5, .06 + .04 * (.5 + .5 * cos (3.*iTime)), .5);

	    float selector = fract(sin(dot(floor(p) + 13.37, vec3(7., 157., 113.)))*43758.5453);

	    if (selector > .75) {
	        p = p;
	    } else if (selector > .5) {
	    	p = p.yzx;
	    } else if (selector > .25) {
		    p = p.zxy;
	    }

	    float d = sdTruchet (opRepeat (p, vec3 (.5)), cellParam);

		return d;
	}

	float raymarch (in vec3 ro, in vec3 rd, out int iter)
	{
	    float t = .0;

	    for (int i = 0; i < MAX_ITER; i++)
	    {
	        iter = i;
	        vec3 p = ro + t*rd;
	        float d = scene (p);
	        if (abs (d) < EPSILON*(1. + .125*d)) break;
	        t += d*STEP_SIZE;
	    }

	    return t;
	}

	vec3 normal (in vec3 p, in float epsilon)
	{
	    vec3 e = vec3(epsilon, .0, .0);
	    float d = scene (p);
	    vec3 n = vec3 (scene (p + e.xyy) - d,
	                   scene (p + e.yxy) - d,
	                   scene (p + e.yyx) - d);
	    return normalize(n);
	}

    float shadow (in vec3 p, in vec3 n, in vec3 lPos)
	{
		float distanceToLight = distance (p, lPos);
		int ignored = 0;
		float distanceToObject = raymarch (p + .01*n,
										   normalize (lPos - p),
										   ignored);
		bool isShadowed = distanceToObject < distanceToLight;
		return isShadowed ? .1 : 1.;
	}

	vec3 shade (in vec3 ro, in vec3 rd, in float d)
	{
	    vec3 p = ro + d * rd;

	    vec3 ambColor = vec3 (.1, .05, .05);
	    vec3 diffColor = vec3 (1.9, 1.4, 1.2);
	    vec3 specColor = vec3 (.95, .85, .85);
	    float shininess = 40.;

	    vec3 lightPos = ro + vec3 (cos (iTime) * .5, .5, sin (iTime) * .5);
	    vec3 lightDir = lightPos - p;
	    vec3 lightNDir = normalize (lightDir);
	    vec3 nor = normal (p, d*EPSILON);
	    vec3 h = normalize (lightDir - rd);

	    float diffuse = max (dot (lightNDir, nor), .0);
	    float specular = pow (max (dot (h, nor), .0), shininess);
		float sha = shadow (p, nor, lightPos);
		float lightDistance = distance (p, lightPos);
		float attenuation = .85 / (lightDistance*lightDistance);
		vec3 diffTerm = sha * attenuation * diffuse * diffColor;
		vec3 specTerm = (sha > .1) ? attenuation * specular * specColor : vec3 (.0);

	    return ambColor + diffTerm + specTerm;
	}

	void main ()
	{
		vec2 uvRaw = fragCoord.xy;
		vec2 uv = uvRaw;
	    uv = uv * 2. - 1.;
	    uv.x *= iResolution.x / iResolution.y;

	    vec3 ro = vec3 (.0, .0, -.75);
	    vec3 rd = normalize (vec3 (uv, 1.) - ro);

	    rd.xz *= rot2d (cos (iTime) * .075);
	    rd.xy *= rot2d (sin (iTime) * .15);

	    int iter = 0;
	    float d = raymarch (ro, rd, iter);
	    float depth = float (iter) / float (MAX_ITER);
	    vec3 cc = gradient (depth).rgb;
	    float fog = 1. / (1. + d * d * .1);
	    vec3 c = fog * shade (ro, rd, d);

		c = c / (1. + c);
	    c = .2 * c + .8 * sqrt (c);
	    c *= vec3 (.9, .8, .7);
	    c *= .2 + .8 * pow (16. * uvRaw.x * uvRaw.y * (1. - uvRaw.x) * (1. - uvRaw.y), .3);

	    if (iMouse.x / iResolution.x < .5) {
			fragColor = vec4(c, 1.);
	    } else {
			fragColor = vec4(cc, 1.);
	    }
	}
