// build/raymarcher-gl chmutov-banchoff-surface.glsl 640 360 30
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

const int MAX_ITER = 64;
const float EPSILON = .0002;
const float STEP_SIZE = .75;

float opCombine (in float d1, in float d2, in float r) {
    float h = clamp (.5 + .5 * (d2 - d1) / r, .0, 1.);
    return mix (d2, d1, h) - r * h * (1. - h);
}

float sdSphere (in vec3 p, in float radius)
{
	return length (p) - radius;
}

float sdPlane (in vec3 p, in float height)
{
	return length (p.y - height);
}

float udBox (in vec3 p, in vec3 size, in float radius)
{
	return length (max (abs (p) - size, .0)) - radius;
}

mat2 r2d (in float degree)
{
    float rad = radians (degree);
    float c = cos (rad);
	float s = sin (rad);

    return mat2 (c, s, -s, c);
}

float scene (in vec3 p)
{
    float ground = sdPlane (p, -1.5);

    vec3 boxCenter = p + vec3 (.0, .5 + .5*cos(iTime), -1.5);
    boxCenter.xz *= r2d (22.*iTime);
    boxCenter.zy *= r2d (-33.*iTime);
    boxCenter.yx *= r2d (43.*iTime);
    float box = udBox (boxCenter, vec3 (.5), .05);
    ground = opCombine (ground, box, .5+.5*cos(4.*iTime));

    p.xz *= r2d (20.*iTime);
    p.zy *= r2d (-30.*iTime);
    float x = p.x*.125;
    float y = p.y*.125;
    float z = p.z*.125;
    float fourthOrderChmutovBanchoffSurface = 3. + 8.*(x*x*x*x + y*y*y*y + z*z*z*z) - 8.*(x*x + y*y + z*z);
    fourthOrderChmutovBanchoffSurface *= .75;
    fourthOrderChmutovBanchoffSurface = opCombine (fourthOrderChmutovBanchoffSurface, ground, .5);

    return fourthOrderChmutovBanchoffSurface;
}

vec3 normal (in vec3 p, in float epsilon)
{
    vec2 e = vec2 (epsilon, .0);
    float d = scene (p);
    return normalize (vec3 (scene (p + e.xyy),
                            scene (p + e.yxy),
                            scene (p + e.yyx)) - d);
}

float raymarch (in vec3 ro, in vec3 rd)
{
    float d = .0;
    float t = .0;
    for (int iter = 0; iter < MAX_ITER; ++iter) {
        t = scene (ro + d * rd);
        if (abs(t) < EPSILON*(1. + .125*t)) break;
        d += t*STEP_SIZE;
    }

    return d;
}

float shadow (in vec3 p, in vec3 lpos)
{
    float distanceToLight = distance (lpos, p);
    vec3 n = normal (p, distanceToLight*EPSILON);
    vec3 ldir = normalize (lpos - p);
    float distanceToObject = raymarch (p + .01 * n, ldir);
    bool isShadowed = distanceToObject < distanceToLight;

	return isShadowed ? .5 : 1.;
}

vec3 shade (in vec3 p, in vec3 n)
{
    vec3 lightPosition1 = vec3 (1.);
    lightPosition1.xz *= r2d (60.*iTime);
    vec3 l1 = normalize (lightPosition1 - p);
    float d1 = distance (p, lightPosition1);
    float lightIntensity1 = 5.;

 	vec3 lightPosition2 = vec3 (1., 1.*cos (2.*iTime), 1.);
    lightPosition2.xz *= r2d (20.*iTime);
    vec3 l2 = normalize (lightPosition2 - p);
    float d2 = distance (p, lightPosition2);
    float lightIntensity2 = 3.;

    vec3 diffuseColor1 = vec3 (.9, .7, .5);
    vec3 diffuseColor2 = vec3 (.5, .7, .9);

    vec3 finalColor1 = max (dot (n, l1), .0) * diffuseColor1 * lightIntensity1 / (d1*d1);
    vec3 finalColor2 = max (dot (n, l2), .0) * diffuseColor2 * lightIntensity2 / (d2*d2);

    return shadow (p, lightPosition1) * finalColor1 +
           shadow (p, lightPosition2) * finalColor2;
}

void main ()
{
    // normalize and aspect-correct
    vec2 uv = fragCoord.xy;
    uv = uv * 2. - 1.;
    uv.x *= iResolution.x/iResolution.y;

    // create viewray
    vec3 ro = vec3 (.0, .0, -2.);
    vec3 rd = normalize (vec3 (uv, .0) - ro);

    // primary/view ray
    float d = raymarch (ro, rd);
	float fog = 1. / (1. + d*d*.025);
    vec3 p = ro + d*rd;
    vec3 n = normal (p, d*EPSILON);
    vec3 col = shade (p, n);

	// secondary/reflection ray
    vec3 refr = normalize (reflect (rd, n));
    float refd = raymarch (p + .001*n, refr);
    vec3 refp = p + refd * refr;
    vec3 refn = normal (refp, EPSILON);
    vec3 refc = shade (refp, refn);
    col += .05*refc;

    // fog, tint, tone-map, gamma-correct
	col *= fog;
    col *= vec3 (.95, .8, .75);
    col = col / (1. + col);
    col = sqrt (col);

    fragColor = vec4 (col, 1.);
}
