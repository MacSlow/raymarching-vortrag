// build/raymarcher-gl /tmp/glass-thing.glsl 1280 720 60 data/rgb-noise-256x256.png

#version 420
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

const int MAX_ITER = 96;
const float EPSILON = .0001;
const float STEP_BIAS = .5;

struct HitResult {
	float dist;
	int materialId;
};

struct Material {
    vec3 diffuse;
    bool doesReflect;
    bool doesRefract;
    float ior;
};

float sdTerrain (in vec3 p, in float height)
{
    float h = texture (iChannel0, .125*p.xz).r;
    h += .5*texture (iChannel0, .25*p.xz).r;
    h += .25*texture (iChannel0, .5*p.xz).r;
    h += .125*texture (iChannel0, 1.*p.xz).r;
    h /= (1. + .5 + .25 + .125);
    h += height;
    return p.y - .0625*h*.25;
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
	float ground = sdTerrain (p, -22.);

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
    vec3 rd = normalize (lPos - p);
    float result = 1.;
    float t = .1;
    float ph = 1e10;
    for (int i = 0; i < 64; i++) {
        float h = scene (p + .01*n + t * rd).dist;
        if (h < .00001) return .0;
        float y = h*h/(2.*ph);
        float d = sqrt (h*h - y*y);
        result = min (result, 10.*d/max (.0, t - y));
        ph = h;
        t += h*.5;
    }

    return result;
}

Material materials[4] = Material[4] (Material (vec3 (1., .1, .2), false, true, 1./1.33),
							         Material (vec3 (.3, 1., .2), true, false, 1.0),
							         Material (vec3 (.75, .85, .95), true, false, 1.0),
							         Material (vec3 (1., .5, .25), false, false, 1.0));

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

	return diffTerm * materials[matId].diffuse + specTerm;
}

vec3 shadeReflect (in vec3 ro,
			       in vec3 rd,
			       in float d,
			       in vec3 n,
			       in int matId,
			       in vec3 lPos,
			       in vec3 lCol)
{
    vec3 base = shade (ro, rd, d, n, matId, lPos, lCol);

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

	return base + diffTerm * materials[matId].diffuse + specTerm;
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
    vec2 uv = fragCoord;
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

    const float airIOR = 1.;
    const float waterIOR = 1.33;
    float rZero = pow (((airIOR - waterIOR) / (airIOR + waterIOR)), 2.);

    vec3 lDir1 = normalize (lPos1 - p);
    vec3 lDir2 = normalize (lPos2 - p);
    float cosTheta1 = max (.0, dot (n, lDir1));
    float cosTheta2 = max (.0, dot (n, lDir2));
    float fresnel1  = rZero + (1. - rZero) * pow((1. - cosTheta1), 5.);
    float fresnel2  = rZero + (1. - rZero) * pow((1. - cosTheta2), 5.);

    if (materials[materialId].doesReflect) {
        vec3 refl = normalize (reflect (rd, n));
        float refd = raymarch (p + .01*n, refl, materialId);
        vec3 refp = p + refd*refl;
        vec3 refn = normal (refp, refd*EPSILON);
        vec3 refc = shade (p, refl, refd, refn, materialId, lPos1, lCol1);
        refc += shade (p, refl, refd, refn, materialId, lPos2, lCol2);

        col += fresnel1*fresnel2*.35*refc;
    }

    if (materials[materialId].doesRefract) {
        vec3 refractedDir = normalize (refract (rd, n, materials[materialId].ior));
        float dist = raymarch (p - .125*n, refractedDir, materialId);
        vec3 refractedHit = p + dist*refractedDir;
        vec3 refractedHitNormal = normal (refractedHit, dist*EPSILON);
        col += shade (p, refractedDir, dist, refractedHitNormal, materialId, lPos1, lCol1);
		col += shade (p, refractedDir, dist, refractedHitNormal, materialId, lPos2, lCol2);

        if (materials[materialId].doesRefract) {
            refractedDir = normalize (refract (refractedDir, refractedHitNormal, materials[materialId].ior));
            dist = raymarch (refractedHit - .125*refractedHitNormal, refractedDir, materialId);
            refractedHit = refractedHit + dist*refractedDir;
            refractedHitNormal = normal (refractedHit, dist*EPSILON);
            col += shade (refractedHit, refractedDir, dist, refractedHitNormal, materialId, lPos1, lCol1);
            col += shade (refractedHit, refractedDir, dist, refractedHitNormal, materialId, lPos2, lCol2);
        }

        if (materials[materialId].doesReflect) {
            vec3 refl = normalize (reflect (refractedDir, refractedHitNormal));
            float refd = raymarch (refractedHit + .01*refractedHitNormal, refl, materialId);
            vec3 refp = refractedHit + refd*refl;
            vec3 refn = normal (refp, refd*EPSILON);
            vec3 refc = shade (refractedHit, refl, refd, refn, materialId, lPos1, lCol1);
            refc += shade (refractedHit, refl, refd, refn, materialId, lPos2, lCol2);
            col += fresnel1*fresnel2*.25*refc;
    	}
    }

	col = mix (col, vec3 (.925, .95, .975), pow (1. - 1. / d, 15.));
	col *= fog;
	col = col / (1. + col);
	col *= vec3 (.875, .85, .8);
	col = .1*col + .9*sqrt (col);
	col *= .2 + .8*pow(16.*uvRaw.x*uvRaw.y*(1. - uvRaw.x)*(1. - uvRaw.y), .3);

    fragColor = vec4 (col, 1.);
}

