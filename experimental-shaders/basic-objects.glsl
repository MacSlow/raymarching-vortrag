// build/raymarcher-gl basic-objects.glsl 640 360 30 data/noise-256x256.png
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

const int MAX_ITER = 48;
const float EPSILON = .001;
const float STEP_BIAS = .95;

struct HitResult {
	float dist;
	int materialId;
};

float sdSphere (in vec3 p, in float r)
{
	return length (p) - r;
}

float sdPlane (in vec3 p, in float h)
{
	return p.y - h;
}

float sdTerrain (in vec3 p, in float h)
{
	float offset = .5*texture (iChannel0, .8*p.xz).r;
	offset += .25*texture (iChannel0, .4*p.xz).r;
	offset += .125*texture (iChannel0, .2*p.xz).r;
	offset += .0625*texture (iChannel0, .1*p.xz).r;
	//offset /= (.5 + .25 + .125 + .0625);
	//offset /= (.5 + .125);
	offset += h;

	return p.y - .05*offset;
}

float sdTorus (in vec3 p, in vec2 t)
{
	vec2 q = vec2 (length (p.xz) - t.x, p.y);
	return length (q) - t.y;
}

float sdBox (in vec3 p, in vec3 size, in float r)
{
  vec3 d = abs(p) - size;
  return min (max (d.x, max (d.y,d.z)), .0) + length (max (d, .0)) - r;
}

float udBox (in vec3 p, in vec3 size, in float r)
{
	return length (max (abs (p) - size, .0)) - r;
}

mat2 r2d (in float degree)
{
	float rad = radians (degree);
	float c = cos (rad);
	float s = sin (rad);

	return mat2 (c, s, -s, c);
}

float sdBoxSphere (in vec3 p, in vec3 size, in float r)
{
	float ball = sdSphere (p, r*8.+r*10.*(.5+.5*cos (2.*iTime)));
	float box = sdBox (p, size, r);

	return  max (ball, box);
}

HitResult scene (in vec3 p)
{
	float ball = sdSphere (p + vec3 (.0, .0, -1.), .3);;
	float ground = sdTerrain (p, -6.);
	float torus = sdTorus (p + vec3 (.4, .1, .0), vec2 (.3, .075));

	vec3 thingCenter = p + vec3 (-.5, -.3, .5);
	float boxSphere = sdBoxSphere (thingCenter, vec3 (.25), .025);

	HitResult result = HitResult (.0, -1);
	result.dist = min (boxSphere, min (torus, min (ground, ball)));
	result.materialId = result.dist == boxSphere ? 0 : (result.dist == torus ? 1 : (result.dist == ground ? 2 : (result.dist == ball ? 3 : -1)));

	return result;
}

float raymarch (in vec3 ro, in vec3 rd, inout int id)
{
	float t = .0;
	float d = .0;
	int i = 0;
	for (; i < MAX_ITER; ++i) {
		HitResult result = scene (ro + t*rd);
		d = result.dist;
		id = result.materialId;
		if (abs (d) < EPSILON*(1. + .125*d)) break;
		t += d*STEP_BIAS;
	}

	return t;
}

vec3 normal (in vec3 p, in float epsilon)
{
	vec2 e = vec2 (epsilon, .0);
	float d = scene (p).dist;
	return normalize (vec3 (scene (p + e.xyy).dist,
							scene (p + e.yxy).dist,
							scene (p + e.yyx).dist) - d);
}

float shadow (in vec3 p, in vec3 n, in vec3 lPos)
{
	float distanceToLight = distance (lPos, p);
	int ignored = 0;
	float distanceToObject = raymarch (p + .01*n, normalize (lPos - p), ignored);
	bool isShadowed = distanceToObject < distanceToLight;
	return isShadowed ? .1 : 1.;
}

vec3 materials[4] = vec3[4] (vec3 (1., .1, .2),
							 vec3 (.3, 1., .2),
							 vec3 (.1, .2, 1.),
							 vec3 (1., .5, .25));

vec3 shade (in vec3 ro,
			in vec3 rd,
			in float d,
			in vec3 n,
			in int matId,
			in vec3 lPos,
			in vec3 lCol)
{
	vec3 p = ro + d*rd;
	vec3 l = normalize (lPos - p);
	float diffuse = max (dot (l, n), .0);
	float sha = shadow (p, n, lPos);
	float lDist = distance (lPos, p);
	float attenuation = 2. / (lDist*lDist);
	vec3 diffTerm = sha*diffuse*lCol*attenuation;

	vec3 h = normalize (l - rd);
	float specular = pow (max (dot (n, h), .0), 20.);
	vec3 specColor = vec3 (1.);
	vec3 specTerm = (sha > .1) ? attenuation*specular*specColor : vec3 (.0);

	return diffTerm * materials[matId] + specTerm;
}

vec3 camera (in vec3 ro, in vec3 aim, in float zoom, in vec2 uv)
{
	vec3 forward = normalize (aim - ro);
	vec3 worldUp = vec3 (.0, 1., .0);
	vec3 right = normalize (cross (forward, worldUp));
	vec3 up = normalize (cross (right, forward));
	vec3 center = ro + forward*zoom;

	return normalize (center + uv.x*right + uv.y*up - ro);
}

void main ()
{
    vec2 uv = fragCoord.xy;
    vec2 uvRaw = uv;
	uv = uv * 2. - 1.;
	uv.x *= iResolution.x/iResolution.y;

	vec3 ro = vec3 (2.*cos (iTime), .5, 2.*sin (iTime));
	vec3 rd = camera (ro, vec3 (.0), 1.5, uv);

	vec3 lPos1 = vec3 (1.);
	vec3 lCol1 = vec3 (.975, .95, .925);
	vec3 lPos2 = vec3 (1., 2., -1.);
	vec3 lCol2 = vec3 (.925, .95, .975);

	int materialId = 0;
	float d = raymarch (ro, rd, materialId);
	float fog = 1. / (1. + d*d*.1);
	vec3 p = ro + d*rd;
	vec3 n = normal (p, d*EPSILON);
	vec3 col = vec3 (.1);
	col += shade (ro, rd, d, n, materialId, lPos1, lCol1);
	col += shade (ro, rd, d, n, materialId, lPos2, lCol2);

	vec3 refl = normalize (reflect (rd, n));
	float refd = raymarch (p + .01*n, refl, materialId);
	vec3 refp = p + refd*refl;
	vec3 refn = normal (refp, refd*EPSILON);
	vec3 refc = shade (p, refl, refd, refn, materialId, lPos1, lCol1);
	refc += shade (p, refl, refd, refn, materialId, lPos2, lCol2);
	col += .15*refc;

	col = mix (col, vec3 (.925, .95, .975), pow (1. - 1. / d, 15.));
	col *= fog;
	col = col / (1. + col);
	col *= vec3 (.875, .85, .8);
	col = .1*col + .9*sqrt (col);
	col *= .2 + .8*pow(16.*uvRaw.x*uvRaw.y*(1. - uvRaw.x)*(1. - uvRaw.y), .3);

    fragColor = vec4 (col, 1.);
}
