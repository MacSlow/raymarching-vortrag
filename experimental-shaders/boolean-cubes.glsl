// build/raymarcher-gl boolean-cubes.glsl 640 360 30 data/noise-64x64.png
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

const int MAX_ITER = 64;
const float EPSILON = .00025;
const float STEP_BIAS = .75;

struct HitResult {
	float dist;
	int materialId;
};

float sdTerrainOrig (in vec3 p, in float h)
{
	float offset = .5*texture (iChannel0, .8*p.xz).r;
	offset += .25*texture (iChannel0, .4*p.xz).r;
	offset += .125*texture (iChannel0, .2*p.xz).r;
	offset += .0625*texture (iChannel0, .1*p.xz).r;
	offset += h;

	return p.y - .05*offset;
}

float sdTerrain (in vec3 p, in float h)
{
	float offset = .5*texture (iChannel0, .1*p.xz).r;
	offset += .25*texture (iChannel0, .2*p.xz).r;
	offset += .125*texture (iChannel0, .4*p.xz).r;
	offset += .0625*texture (iChannel0, .8*p.xz).r;
	offset /= (.5 + .25 + .125 + .0625);
	offset += h;

	return p.y - .2*offset;
}

float sdBox (in vec3 p, in vec3 size, in float r)
{
  vec3 d = abs(p) - size;
  return min (max (d.x, max (d.y,d.z)), .0) + length (max (d, .0)) - r;
}

mat2 r2d (in float degree)
{
	float rad = radians (degree);
	float c = cos (rad);
	float s = sin (rad);

	return mat2 (c, s, -s, c);
}

HitResult scene (in vec3 p)
{
	float ground = sdTerrain (p, -3.);

	vec3 boxCenter = p + vec3 (.0, -.5, .0);
	boxCenter.xy *= r2d(60.*iTime);
	boxCenter.yz *= r2d(-30.*iTime);
	boxCenter.zx *= r2d(40.*iTime);
	float outterBox = sdBox (boxCenter, vec3 (.4), .05);
	float innerBox = sdBox (boxCenter, vec3 (.375), .05);
	outterBox = max (-innerBox, outterBox);
	
	vec3 cutterBoxCenter = p + vec3 (.0, -.2, .0);
	cutterBoxCenter.xy *= r2d(20.*iTime);
	cutterBoxCenter.yz *= r2d(40.*iTime);
	cutterBoxCenter.y += .4*cos (iTime);
	cutterBoxCenter.y = mod (cutterBoxCenter.y + .1, .2) - .1;
	float cutterBoxes = sdBox (cutterBoxCenter, vec3 (1., .035, 1.), .0);
	outterBox = max (outterBox, cutterBoxes);

	vec3 ballCenter = p + vec3 (.0, -.5, .0);
	float warp = .2 + .1*(.5+.5*cos (20.*ballCenter.y + 5.*iTime));
	float ball = length (ballCenter) - warp;

	HitResult result = HitResult (.0, -1);
	result.dist = min (ball, min (outterBox, ground));
	result.materialId = result.dist == outterBox ? 0 : (result.dist == ground ? 2 : (result.dist == ball ? 1 : -1));

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
							 vec3 (.75, .85, .95),
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

	vec3 ro = vec3 (2.*cos (iTime), .7, 2.*sin (iTime));
	vec3 rd = camera (ro, vec3 (.0, .3, .0), 1.5, uv);

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
