#version 420
uniform vec3 iResolution;
uniform float iTime;
uniform float iTimeDelta;
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

const float ESCAPE_DISTANCE      = 10.;
const float BOUNDING_DOMAIN_SIZE = 3.;
const float DEL                  = .00001; 
const int MAX_ITERATIONS         = 9;
const int INTERSECTION_DEPTH     = 96;
const float EPSILON              = .003;
const bool SHADOWS               = true;

vec4 quatMult (vec4 q1, vec4 q2)
{
	vec4 r;
	r.x   = q1.x*q2.x - dot (q1.yzw, q2.yzw);
	r.yzw = q1.x*q2.yzw + q2.x*q1.yzw + cross (q1.yzw, q2.yzw);
	return r;
}

vec4 quatSq (vec4 q)
{
	vec4 r;
	r.x   = q.x*q.x - dot (q.yzw, q.yzw);
	r.yzw = 2.*q.x*q.yzw;
	return r;
}

void iterateIntersect (inout vec4 q, inout vec4 qp, vec4 c)
{
    for (int i = 0; i < MAX_ITERATIONS; i++) {
	    qp = 2.0 * quatMult(q, qp);
    	q = quatSq (q) + c;
    	if (dot (q, q) > ESCAPE_DISTANCE) {
            break;
        }
    }
}

vec3 normal (vec3 p, vec4 c)
{
	vec3 N;
    vec4 qP = vec4 (p, .0);
	float gradX;
	float gradY;
	float gradZ;
	vec4 gx1 = qP - vec4 (DEL, .0, .0, .0);
	vec4 gx2 = qP + vec4 (DEL, .0, .0, .0);
    vec4 gy1 = qP - vec4 (.0, DEL, .0, .0);
    vec4 gy2 = qP + vec4 (.0, DEL, .0, .0);
    vec4 gz1 = qP - vec4 (.0, .0, DEL, .0);
    vec4 gz2 = qP + vec4 (.0, .0, DEL, .0);

    for (int i = 0; i < MAX_ITERATIONS; i++) {
		gx1 = quatSq (gx1) + c;
		gx2 = quatSq (gx2) + c;
		gy1 = quatSq (gy1) + c;
		gy2 = quatSq (gy2) + c;
		gz1 = quatSq (gz1) + c;
		gz2 = quatSq (gz2) + c;
	}

	gradX = length (gx2) - length (gx1);
	gradY = length (gy2) - length (gy1);
	gradZ = length (gz2) - length (gz1);
	N = normalize (vec3 (gradX, gradY, gradZ));
	return N;
}

float intersectQJulia (inout vec3 ro, inout vec3 rd, vec4 c, float epsilon)
{
	float dist;
    float dummy = .0;
    for (int i = 0; i < INTERSECTION_DEPTH; ++i)
	{
		vec4 z = vec4 (ro, .0);
		vec4 zp = vec4 (1., .0, .0, .0);
		iterateIntersect (z, zp, c);

		float normZ = length (z);
		dist = .5 * normZ * log (normZ) / length (zp);
		ro += rd * dist;
        if (dist < epsilon || dot (ro, ro) > BOUNDING_DOMAIN_SIZE) {
	        break;
        }
	}

	return dist;
}

vec3 shade (vec3 light, vec3 eye, vec3 pt, vec3 N)
{
	vec3 diffuse = vec3 (1., .45, .25);
	const float specularExponent = 10.;
	const float specularity = .45;
	vec3 L = normalize (light - pt);
    vec3 E = normalize (eye - pt);
	float NdotL = dot (N, L);
	vec3 R = L - 2. * NdotL * N;
	diffuse += abs (N) * .3;

    return diffuse * max (NdotL, .0) + specularity * pow (max (dot (E,R),.0), specularExponent);
}

vec3 intersectSphere (vec3 ro, vec3 rd)
{
	float B;
	float C;
	float d;
	float t0;
	float t1;
	float t;
	B = 2. * dot (ro, rd);
	C = dot (ro, ro) - BOUNDING_DOMAIN_SIZE;
	d = sqrt (B * B - 4. * C);
	t0 = (-B + d) * .5;
	t1 = (-B - d) * .5;
	t = min (t0, t1);
	ro += t * rd;
	return ro;
}

mat3 camera (vec3 ro, vec3 ta, float cr)
{
	vec3 cw = normalize (ta - ro);
	vec3 cp = vec3 (sin (cr), cos (cr), .0);
	vec3 cu = normalize (cross (cw, cp));
	vec3 cv = normalize (cross (cu, cw));
    return mat3 (cu, cv, cw);
}

void main ()
{
	vec2 uvRaw = fragCoord.xy;
	vec2 uv = uvRaw;
	uv = uv * 2 - 1;
	uv *= vec2 (iResolution.x / iResolution.y, 1.);
	uvRaw *= (iResolution.x / iResolution.y, 1.);

	vec2 mouse = iMouse.xy / iResolution.xy;
    vec3 eye = vec3 (.5 + 2.5 * cos (6. * mouse.x),
					 -2. + 4. * sin (mouse.y),
					 .5 + 2.5 * sin (6. * mouse.x));
    vec3 aim = vec3 (.0, .0, .0);
    mat3 ca = camera (eye, aim, .0);
    float u = (fragCoord.x * 2. - 1.) * iResolution.z;
    float v = fragCoord.y * 2. - 1.;
	vec3 ro = eye;
	vec3 rd = normalize (ca * vec3 (u, -v, 2.));

    vec4 mu = vec4 (-.2, .6, .5 * cos (iTime), .125 * sin (iTime));
    vec3 light1 = vec3 (cos (iTime) * 2., 12., sin (iTime) * 15.);
    vec3 light2 = vec3 (cos (iTime) * (-3.), -6., sin (iTime) * (-10.));
	const vec4 backgroundColor = vec4 (.3, .3, .3, .0);
	vec4 color;

	color = backgroundColor;
	rd = normalize (rd);
	ro = intersectSphere (ro, rd);

    float dist = intersectQJulia (ro, rd, mu, EPSILON);
	if(dist < EPSILON) {
		vec3 N = normal (ro, mu);
		color.rgb = shade (light1, rd, ro, N);
		color.rgb += shade (light2, rd, ro, N);
		color.a = 1.;
		if (SHADOWS) {
			vec3 L = normalize (light1 - ro);
			ro += N * EPSILON * 2.;

			dist = intersectQJulia (ro, L, mu, EPSILON);
            if (dist < EPSILON) {
				color.rgb *= .4;
            }
		}
	}

	// gamma-correction, tint, vingette
	color.rgb = .2 * color.rgb + .8 * sqrt (color.rgb);
	color.rgb *= vec3 (.9, .8, .7);
    color.rgb *= .2 + .8 * pow (16. * uvRaw.x * uvRaw.y * (1. - uvRaw.x) * (1. - uvRaw.y), .3);

	fragColor = color;
}

