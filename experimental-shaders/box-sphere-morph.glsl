// build/raymarcher-gl box-sphere-morph.glsl 640 360 30 data/noise-64x64.png
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

const int MAX_ITER = 48;
const float EPSILON = .0005;
const float STEP_BIAS = .75;

mat2 r2d (in float degree) {
	float c = cos (radians (degree));
    float s = sin (radians (degree));
    return mat2 (c, s, -s, c);
}

float sdSphere (in vec3 p, in float r)
{
	return length (p) - r;
}

float sdBox (in vec3 p, in vec3 size, in float r)
{
  vec3 d = abs(p) - (size - r);
  return min (max (d.x, max (d.y,d.z)), .0) + length (max (d, .0)) - r;
}

float sdTerrain (in vec3 p, in float height)
{
    float h = .5*texture (iChannel0, .1*p.xz).r;
    h += .25*texture (iChannel0, .2*p.xz).r;
    h += .125*texture (iChannel0, .4*p.xz).r;
    h += .0625*texture (iChannel0, .8*p.xz).r;
    h /= (.5 + .25 + .125 + .0625);
    h += height;
    return p.y - .2*h;
}

float sdTorus (in vec3 p, in vec2 t)
{
    vec2 q = vec2 (length (p.xz) - t.x, p.y);
    return length (q) - t.y;
}

float sdBoxSphere (in vec3 p, in float size, in float phase)
{
	float box = sdBox (p, vec3 (size), size*.1);
	float factor = mix (.0, 1.85, phase);
	float sphere = sdSphere (p, size*factor);
	return max (sphere, box);	
}

float opRepeat (inout float p, in float size)
{
	float hsize = .5*size;
	float cell = floor ((p + hsize) / size);
	p = mod (p + hsize, size) - hsize;
	return cell;
}

vec2 opRepeat (inout vec2 p, in float size)
{
	vec2 hsize = vec2 (.5*size);
	vec2 cell = floor ((p + hsize) / size);
	p = mod (p + hsize, size) - hsize;
	return cell;
}

vec3 opRepeat (inout vec3 p, in float size)
{
	vec3 hsize = vec3 (.5*size);
	vec3 cell = floor ((p + hsize) / size);
	p = mod (p + hsize, size) - hsize;
	return cell;
}

struct HitResult {
	float dist;
	int id;
};

// map 'value' from range a..b to p..q
float map (float a, float b, float p, float q, float value)
{    
    float vn = (value - a) / (b - a);
    float delta = q - p; 
    return p + vn * delta;
}    

HitResult scene (in vec3 p) {
    vec3 torusCenter = p + vec3 (.0, -1.5, .0);
	torusCenter.xy *= r2d (30.*iTime);
	torusCenter.yz *= r2d (-60.*iTime);
	torusCenter.zx *= r2d (45.*iTime);
    float torus = sdTorus (torusCenter, vec2 (1., .375));

    vec3 boxCenter = p + vec3 (.0, -1., .0);
	boxCenter.xy *= r2d (-60.*iTime);
	boxCenter.yz *= r2d (40.*iTime);
	boxCenter.zx *= r2d (75.*iTime);
	vec3 cell = opRepeat (boxCenter, .275);
	float phase = .5 + .5*cos (.75*(cell.x + cell.y + cell.z) + iTime);
    float box = sdBoxSphere (boxCenter, .125, phase);

	box = max (torus, box);

    float ground = sdTerrain (p, -1.);

	HitResult result = HitResult (.0, -1);
	result.dist = min (ground, box);
	result.id = result.dist == ground ? 1 : (result.dist == box ? 2 : -1);

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

Material[3] materials = Material[3] (Material (vec3 (1., .5, .2), 10.),
							         Material (vec3 (.5, 1., .2), 50.),
							         Material (vec3 (.2, .5, 1.), 150.));

vec3 shade (in vec3 ro, in vec3 rd, in float d, in int matId) {
    vec3 p = ro + d*rd;
    vec3 n = normal (p, d*EPSILON);
    vec3 amb = vec3 (.15);
	vec3 color = matId == 1 ? mix (vec3 (.9),
                                   vec3 (1., .1, .1),
                                   smoothstep (.475, .5, cos(2.*p.x)*cos (2.*p.z))) :
                              materials[matId].color;
	float mask = mix (1., .0, smoothstep (.3, .5, (.5+.5*sin (60.*p.x))*(.5+.5*sin(60.*p.y))));
	float shininess = matId == 0 ? mix (20., 60., mask) : materials[matId].shininess;
    vec3 term1 = diffSpecTerm (ro, rd, d,
                              vec3 (2.*cos (2.*iTime), 3., -1. + 2.*sin (1.5*iTime)),
                              color, 5., vec3 (1.), shininess);

    vec3 term2 = diffSpecTerm (ro, rd, d,
                              vec3 (1. + 2.*cos (2.*iTime), 3., -1. + 2.*sin (2.*iTime)),
                              color, 4., vec3 (1.), shininess);

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
    // coordinate-normalization, aspect-correction
    vec2 uv = fragCoord.xy;
    vec2 uvRaw = uv;
	uv = uv * 2. - 1.;
	uv.x *= iResolution.x/iResolution.y;

    // "camerea"/view setup
    float offset = 2.5;
    vec3 ro = vec3 (offset*cos (iTime), 3., offset*sin (iTime));
    vec3 aim = vec3 (.0, .75, .0);
    float zoom = 1.;
    vec3 rd = camera (uv, ro, aim, zoom);

    // primary/view-ray
    HitResult result = raymarch (ro, rd);
    float d = result.dist;
    float fog = 1. / (1. + d*.025);
    vec3 p = ro + d*rd;
    vec3 n = normal (p, d*EPSILON);
    vec3 col = shade (ro, rd, d, result.id);

    // secondary/1st reflection-ray
    vec3 refl = normalize (reflect (rd, n));
    result = raymarch (p + .01*n, refl);
    float refd = result.dist;
    vec3 refp = p + refd*refl;
	vec3 refn = normal (refp, refd*EPSILON);
    vec3 refcol = shade (p, refl, refd, result.id);

    // ternary/2st reflection-ray
    vec3 refl2 = normalize (reflect (refl, refn));
    result = raymarch (refp + .01*refn, refl2);
    float refd2 = result.dist;;
    vec3 refp2 = refp + refd2*refl2;
    vec3 refcol2 = shade (refp, refl2, refd2, result.id);

    // restricting reflections to grazing view-angles
	float fakeFresnel = 1.;//pow (1. - max (dot (n, -rd), .0), 1.25);
    col += fakeFresnel*.5*refcol;
    col += fakeFresnel*.5*refcol2;

    // fog, "horizon", tint, tone-mapping, gamma-correction, vignette
    col *= fog;
    col = mix (col, vec3 (.65, .75, .85), pow (1. - 1. / d, 30.));
    col *= vec3 (.9, .8, .7);
    col = col / (1. + col);
    col = .3 * col + .7 * sqrt (col);
    col *= .3 + .7 * pow (16. * uvRaw.x * uvRaw.y * (1. - uvRaw.x) * (1. - uvRaw.y), .2);

    fragColor = vec4 (col, 1.);
}
