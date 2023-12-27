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

const int MAX_ITER    = 64;
const float STEP_SIZE = .975;
const float EPSILON   = .001;

struct Result {
	float d;
	int id;
};

float sdPlane (in vec3 p, in float h)
{
	return p.y - h;
}

float sdSphere (in vec3 p, in float r)
{
	return length (p) - r;
}

float sdCylinder (in vec2 p, in float r)
{
	return length (p) - r;
}

float sdBox (in vec3 p, in vec3 size)
{
	return length (max (abs (p) - size, .0));
}

Result scene (in vec3 p)
{
    float floor = sdPlane (p, -.4);

	vec3 sphereCenter = p;
	vec3 boxCenter = p;
	vec3 cylinderCenter = p;
	sphereCenter -= vec3 (1.2, .2 + .4*(.5+.5*cos(2.5*iTime)), .0);
	boxCenter -= vec3 (-1.2, -.1, .25 + .5*cos (3.*iTime));
	cylinderCenter -= vec3 (.2*cos(2.*iTime), .0, .0);

	float sphere = sdSphere (sphereCenter, .6);
	float box = sdBox (boxCenter, vec3 (.3, .3, .4));
	float cylinder = sdCylinder (cylinderCenter.xz, .25);
    float d = min (cylinder, min (box, sphere));

    Result res = Result (.0, 0);
	res.d = min (d, floor);
    res.id = (res.d == floor ) ? 1 : 2;

    return res;
}

vec3 normal (in vec3 p)
{
    vec2 e = vec2(EPSILON, .0);
    float d = scene (p).d;
    vec3 n = vec3 (scene (p + e.xyy).d - d,
                   scene (p + e.yxy).d - d,
                   scene (p + e.yyx).d - d);
    return normalize(n);
}

Result raymarch (in vec3 ro, in vec3 rd)
{
    Result res = Result (.0, 0);

    for (int i = 0; i < MAX_ITER; i++)
    {
        vec3 p = ro + res.d * rd;
        Result tmp = scene (p);
        if (tmp.d < EPSILON) return res;
        res.d += tmp.d * STEP_SIZE;
        res.id = tmp.id;
    }

    return res;
}

vec3 shadeLambert (in vec3 ro, in vec3 rd, in float d, in int id)
{
    vec3 p = ro + d * rd;
    vec3 nor = normal (p);

    vec3 lightColor = vec3 (.8, .8, .9) * 20.;
    vec3 lightPosition = p + vec3 (.5, .75, -1.5);
    vec3 lightDir = normalize (lightPosition - p);
    vec3 diffuse = (id == 1) ? vec3 (.9) : vec3 (.05, .65, .05);

	float distanceToLight = distance (lightPosition, p);
	float distanceToObject = raymarch (p + .01*nor, lightDir).d;
	bool isShadowed = distanceToObject < distanceToLight;

    vec3 shadingColor = diffuse * max (dot (nor, lightDir), .0) * lightColor;
	vec3 finalColor = shadingColor * (isShadowed ? .1 : 1.);

    return finalColor;
}

vec3 camera (in vec2 uv, in vec3 ro, in vec3 aim, in float zoom)
{
    vec3 camForward = normalize (vec3 (aim - ro));
    vec3 worldUp = vec3 (.0, 1., .0);
    vec3 camRight = normalize (cross (camForward, worldUp));
    vec3 camUp = normalize (cross (camRight, camForward));
    vec3 camCenter = normalize (ro + camForward * zoom);

    return normalize ((camCenter + uv.x*camRight + uv.y*camUp) - ro);
}

vec3 correctColor (in vec3 c, in float fog, in vec2 uvRaw)
{
	vec3 color = c * fog;
	color= color / (1. + color);
    color = .2 * color + .8 * sqrt (color);
    color *= vec3 (.9, .8, .7);
    color *= .2 + .8*pow(16.*uvRaw.x*uvRaw.y*(1. - uvRaw.x)*(1. - uvRaw.y), .3);

	return color;
}

void main ()
{
    // normalizing and aspect-correction
	vec2 uvRaw = fragCoord.xy;
	vec2 uv = uvRaw;
    uv = uv * 2. - 1.;
    uv.x *= iResolution.x / iResolution.y;

    // set up "camera", view origin (ro) and view direction (rd)
    vec3 ro = vec3 (0.0, 2.0, -5.0);
    vec3 aim = vec3 (0.0, 2.0, 0.0);
    float zoom = 1.;
    vec3 rd = camera (uv, ro, aim, zoom);

    // do the ray-march...
    Result res = raymarch (ro, rd);
    float fog = 1. / (1. + res.d * res.d * .1);
    vec3 c = shadeLambert (ro, rd, res.d, res.id);

	c = correctColor (c, fog, uvRaw);

	fragColor = vec4(c, 1.);
}
