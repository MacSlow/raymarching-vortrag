// build/raymarcher-gl weird-torus.glsl 640 360 30 data/noise-64x64.png
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

precision highp float;

const int MAX_ITER = 48;
const float EPSILON = .001;
const float STEP_BIAS = .75;

float saturate (in float v) { return clamp (v, .0, 1.); }

mat2 r2d (in float a) {
	float c = cos (radians (a));
    float s = sin (radians (a));
    return mat2 (c, s, -s, c);
}

float sdSphere (in vec3 p, in float r) {
    return length (p) - r;
}

float sdBox (in vec3 p, in vec3 size, in float r)
{
  vec3 d = abs(p) - size;
  return min (max (d.x, max (d.y,d.z)), .0) + length (max (d, .0)) - r;
}

float sdTerrain (in vec3 p, in float height)
{
    float h = .5*texture (iChannel0, .1*p.xz).r;
    h += .25*texture (iChannel0, .2*p.xz).r;
    h += .125*texture (iChannel0, .4*p.xz).r;
    h += .0625*texture (iChannel0, .8*p.xz).r;
    //h /= (.5 + .25 + .125 + .0625);
    h += height;
    return p.y - .125*h;
}

float sdTorus (in vec3 p, in vec2 t)
{
    vec2 q = vec2 (length (p.xz) - t.x, p.y);
    return length (q) - t.y;
}

float sdBoxSphere (in vec3 p, in float size, in float phase)
{
	float r = size / 20.;
	float box = sdBox (p, vec3 (size), r);
	float sphere = sdSphere (p, r*mix (21., 36., phase));
	return max (box, sphere);
}

vec3 opRepeat (inout vec3 p, in float size)
{
	float hsize = .5*size;
	vec3 cell = floor ((p + hsize)/vec3(size));
	p = mod (p + hsize, vec3 (size)) - hsize;
	return cell;
}

struct HitResult {
	float dist;
	int id;
};

HitResult scene (in vec3 p) {
    vec3 boxSphereCenter = p + vec3 (.0, -1., .0);
    vec3 torusCutterCenter = boxSphereCenter;
    boxSphereCenter.xy *= r2d (10.*iTime);
    boxSphereCenter.yz *= r2d (15.*iTime);
    boxSphereCenter.zx *= r2d (20.*iTime);
    vec3 cell = opRepeat (boxSphereCenter, .75*.8 + .2*(.5+.5*cos (1.75*iTime)));
	float boxSphere = sdBoxSphere (boxSphereCenter, .2, .5+.5*cos (3.*sin(cell.x+cell.y+cell.z)+2.*iTime));
	float torusCutter = sdTorus (torusCutterCenter, vec2 (1.5, .6));
	boxSphere = max (torusCutter, boxSphere);
	int id = int (floor (mod (cell.x+cell.y+cell.z, 4.)));

    float ground = sdTerrain (p, -1.);

	HitResult result = HitResult (.0, -1);
	result.dist = min (ground, boxSphere);
	result.id = result.dist == ground ? 1 : ( result.dist == boxSphere ? id : -1);

    return result;
}

HitResult raymarch (in vec3 ro, in vec3 rd) {
    float d = .0;
	float t = .0;
	HitResult result;
    for (int i = 0; i < MAX_ITER; ++i) {
        result = scene (ro + t*rd);
        d = result.dist;
        if (abs (d) < EPSILON * (1. + .125*d)) break;
        t += d*STEP_BIAS;
    }

	result.dist = t;
    return result;
}

vec3 normal (in vec3 p, in float epsilon) {
    float d = scene (p).dist;
    vec2 e = vec2 (epsilon, .0);
    return normalize (vec3 (scene (p + e.xyy).dist,
                            scene (p + e.yxy).dist,
                            scene (p + e.yyx).dist) - d);
}

float shadow (in vec3 p, in vec3 n, in vec3 lPos) {
    float lDist = distance (p, lPos);
    vec3 lDir = normalize (lPos - p);
    float dist = raymarch (p + .01*n, lDir).dist;
    return dist < lDist ? .1 : 1.;
}

vec3 diffSpecTerm (in vec3 ro,
                   in vec3 rd,
                   in float d,
                   in vec3 lPos,
                   in vec3 diffColor,
                   in float attenuationDamping,
                   in vec3 specColor,
				   in float shininess) {
    vec3  p = ro + d*rd;
    vec3  n = normal (p, d*EPSILON);
    vec3  lDir = normalize (lPos - p);
    float lDist = distance (lPos, p);
    float diff = max (dot (lDir, n), .0);
    float sha = shadow (p, n, lPos);
    float attenuation = attenuationDamping / (lDist*lDist);
    vec3  diffTerm = sha*attenuation*diff*diffColor;
    vec3  h = normalize (lDir - rd);
    float spec = pow (max (dot (n, h), .0), shininess);
    vec3  specTerm = (sha > .1) ? attenuation*spec*specColor : vec3 (.0);

    return diffTerm + specTerm;
}

struct Material {
	vec3 color;
	float shininess;
};

Material[4] materials = Material[4] (Material (vec3 (1., .5, .2), 10.),
							         Material (vec3 (.5, 1., .2), 50.),
							         Material (vec3 (.2, .5, 1.), 150.),
							         Material (vec3 (.3, .8, .4), 30.));

vec3 shade (in vec3 ro, in vec3 rd, in float d, in int matId) {
    vec3 p = ro + d*rd;
    vec3 n = normal (p, d*EPSILON);
    vec3 amb = vec3 (.15);
	vec3 color = matId == 1 ? mix (vec3 (.9),
								   vec3 (1., .1, .1),
								   smoothstep (.475, .5, cos(p.x)*cos (p.z))) :
							  materials[matId].color;
	
	float mask = mix (1., .0, smoothstep (.3, .5, (.5+.5*sin (60.*p.x))*(.5+.5*sin(60.*p.y))));
	float shininess = matId == 0 ? mix (20., 60., mask) : materials[matId].shininess;
    vec3 term1 = diffSpecTerm (ro, rd, d,
                              vec3 (2.*cos (2.*iTime), 3., -1. + 2.*sin (1.5*iTime)),
                              color, 3., vec3 (1.), shininess);

    vec3 term2 = diffSpecTerm (ro, rd, d,
                              vec3 (1. + 2.*cos (2.*iTime), 3., -1. + 2.*sin (2.*iTime)),
                              color, 2., vec3 (1.), shininess);

    return amb + term1 + term2;

}

vec3 camera (in vec2 uv, in vec3 ro, in vec3 aim, in float zoom) {
    vec3 camForward = normalize (vec3 (aim - ro));
    vec3 worldUp = vec3 (.0, 1., .0);
    vec3 camRight = normalize (cross (worldUp, camForward));
    vec3 camUp = normalize (cross (camForward, camRight));
    vec3 camCenter = ro + camForward * zoom;

    return normalize (camCenter + uv.x * camRight + uv.y * camUp - ro);
}

void main ()
{
    vec2 uv = fragCoord.xy;
    vec2 uvRaw = uv;
	uv = uv * 2. - 1.;
	uv.x *= iResolution.x/iResolution.y;

    vec3 ro = vec3 (.0 + 3.*cos (.5*iTime), 3., .5 + 3.*sin (.5*iTime));
    vec3 aim = vec3 (.0);
    float zoom = 1.125;
    vec3 rd = camera (uv, ro, aim, zoom);

    HitResult result = raymarch (ro, rd);
    float d = result.dist;
    float fog = 1. / (1. + d*.05);
    vec3 p = ro + d*rd;
    vec3 n = normal (p, d*EPSILON);
    vec3 col = shade (ro, rd, d, result.id);

    vec3 refl = normalize (reflect (rd, n));
    result = raymarch (p + .01*n, refl);
    float refd = result.dist;
    vec3 refp = p + refd*refl;
	vec3 refn = normal (refp, refd*EPSILON);
    vec3 refcol = shade (p, refl, refd, result.id);

    vec3 refl2 = normalize (reflect (refl, refn));
    result = raymarch (refp + .01*refn, refl2);
    float refd2 = result.dist;;
    vec3 refp2 = refp + refd2*refl2;
    vec3 refcol2 = shade (refp, refl2, refd2, result.id);

	float fakeFresnel = pow (1. - max (dot (n, -rd), .0), 1.25);
    col += fakeFresnel*.85*refcol;
    col += fakeFresnel*.85*refcol2;

    col *= fog;
    col = mix (col, vec3 (.65, .75, .85), pow (1. - 1. / d, 30.));
    col *= vec3 (.9, .8, .7);
    col = col / (1. + col);
    col = .3 * col + .7 * sqrt (col);
    col *= .3 + .7 * pow (16. * uvRaw.x * uvRaw.y * (1. - uvRaw.x) * (1. - uvRaw.y), .2);

    fragColor = vec4 (col, 1.);
}

