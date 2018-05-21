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
const float EPSILON = .0002;

void pR45 (inout vec2 p)
{
	p = (p + vec2(p.y, -p.x))*sqrt(0.5);
}

float pMod1 (inout float p, in float size)
{
	float halfsize = size*.5;
	float c = floor ((p + halfsize)/size);
	p = mod (p + halfsize, size) - halfsize;
	return c;
}

float opUnionColumns (in float a, in float b, in float r, in float n)
{
	if ((a < r) && (b < r)) {
		vec2 p = vec2(a, b);
		float columnradius = r*sqrt(2.)/((n - 1.)*2. + sqrt(2.));
		pR45 (p);
		p.x -= sqrt(2.)/2.*r;
		p.x += columnradius*sqrt(2.);
		if (mod(n,2.) == 1.) {
			p.y += columnradius;
		}

		pMod1 (p.y, columnradius*2.);
		float result = length(p) - columnradius;
		result = min (result, p.x);
		result = min (result, a);
		return min (result, b);
	} else {
		return min (a, b);
	}
}

float opUnionChamfer(in float a, in float b, in float r)
{
	return min (min (a, b), (a - r + b)*sqrt (.5));
}

float opUnionSmooth (in float d1, in float d2, in float r)
{
    float h = clamp (.5 + .5 * (d2 - d1) / r, .0, 1.);
    return mix (d2, d1, h) - r * h * (1. - h);
}

float udBox (in vec3 p, in vec3 size, in float r)
{
    return length (max (abs (p) - (size -r), .0)) - r;
}

float sdBox (in vec3 p, in vec3 size, in float r)
{
  vec3 d = abs(p) - size;
  return min (max (d.x, max (d.y,d.z)), .0) + length (max (d, .0)) - r;
}

float sdSphere (in vec3 p, in float r)
{
    return length (p) - r;
}

float opRepeat1 (in float p, float d)
{
    return mod (p + d*.5, d) - d*.5;
}

vec2 opRepeat2 (in vec2 p, float d)
{
    return mod (p + d*.5, d) - d*.5;
}

vec3 opRepeat3 (in vec3 p, float d)
{
    return mod (p + d*.5, d) - d*.5;
}

float opUnion (in float a, in float b)
{
	return min (a, b);
}

float opIntersect (in float a, in float b)
{
	return max (a, b);
}

float opSubtract (in float a, in float b)
{
	return max (-a, b);
}

mat3 rotY (in float a) {float c = cos(a); float s = sin (a); return mat3 (vec3 (c, .0, s), vec3 (.0, 1., .0), vec3 (-s, .0, c));}

float udRoundBox (in vec3 p, in vec3 size, in float r) { return length (max (abs (p) - size, .0)) - r; }

float twoPlusSigns (in vec3 p)
{
    float verticalBar1 = udRoundBox (p, vec3 (.1, .1, .3), .0125);
    float horizontalBar1 = udRoundBox (p, vec3 (.1, .3, .1), .0125);
    p += vec3 (.0, .0, .9);
    float verticalBar2 = udRoundBox (p, vec3 (.1, .1, .3), .0125);
    float horizontalBar2 = udRoundBox (p, vec3 (.1, .3, .1), .0125);
    float plusplus = opUnion (verticalBar1, horizontalBar1);

    return opUnion (plusplus, opUnion (verticalBar2, horizontalBar2));
}

float sdCylinder (in vec3 p, in vec2 size)
{
    vec2 d = abs (vec2 (length (p.xz), p.y)) - size;
    return min (max (d.x, d.y), .0) + length (max (d, .0));
}

float scene (in vec3 p)
{
    vec3 center = (p+vec3(.125, .0, -.2)) * rotY (-iTime);
    float plusSigns = twoPlusSigns (center);

    vec3 cCenter = center.yxz + vec3 (.0, .0, -1.);
    float cylinder1 = sdCylinder (cCenter, vec2 (.6, .1));
    float cylinder2 = sdCylinder (cCenter, vec2 (.4, .25));
    float cylinder3 = sdCylinder (cCenter + vec3 (.0, .0, .3),
                                  vec2 (.4, .25));
    cCenter += vec3 (.0, .0, .4125);
    float cutBox2 = sdBox (cCenter * rotY (radians (45.)),
                           vec3 (.35, .15, .35), .0);
    float f = opUnion (cylinder2, cutBox2);
    float cRing = opSubtract (opUnion (cylinder2, cutBox2), cylinder1);
    float cplusplus = opUnion (plusSigns, cRing);

    vec3 p1 = p + vec3 (.355);
    p1 = opRepeat3 (p1, .35);
    float boxes = udBox (p1, vec3 (.15), .02);
    float cutBox = sdBox (p, vec3 (1.85), .05);
    float wallBox = sdBox (p, vec3 (1.95), .05);

    return opUnion (cplusplus,
                    opUnion (-wallBox,
                             opSubtract (cutBox, boxes)));
}

float raymarch (in vec3 ro, in vec3 rd)
{
    float t = .0;
    float d = .0;
    for (int i = 0; i < MAX_ITER; ++i) {
        vec3 p = ro + d * rd;
        t = scene (p);
        //if (t < EPSILON) break;
        if (abs(t) < EPSILON*(1. + .125*t)) break;
        d += t*.5;
    }

    return d;
}

vec3 normal (in vec3 p, in float epsilon)
{
    const vec2 e = vec2 (epsilon, .0);
    return normalize (vec3 (scene (p + e.xyy),
                            scene (p + e.yxy),
                            scene (p + e.yyx)) - scene (p));
}

vec3 shade (in vec3 ro, in vec3 rd, in float d)
{
    vec3 p = ro + d * rd;
    const vec3 ambient = vec3 (.05);
    const vec3 diffuseColor = vec3 (.9, .3, .3);
    const vec3 specularColor = vec3 (.9, .8, .7);
    const float shininess = 40.;
    const float diffuseStrength = .25;
    float t = 3.*iTime;

    vec3 n = normal (p, d*EPSILON);
    vec3 lPos = vec3 (cos (t), 1., sin (t));
    float lDist = distance (lPos, p);
    vec3 lDir = normalize (lPos - p);
    vec3 hDir = normalize (ro + lDir);
    float diffuse = max (dot (n, lDir), .0)*(1. / lDist)*diffuseStrength;
    float specular = pow (max (dot (hDir, n), .0), shininess);

	const vec3 diffuseColor2 = vec3 (.3, .9, .3);
    const vec3 specularColor2 = vec3 (.7, .8, .9);
    vec3 lPos2 = vec3 (.0, sin(t), .75*cos(t));
    float lDist2 = distance (lPos2, p);
    vec3 lDir2 = normalize (lPos2 - p);
    vec3 hDir2 = normalize (ro + lDir2);
    float diffuse2 = max (dot (n, lDir2), .0)*(1. / lDist2)*diffuseStrength;
    float specular2 = pow (max (dot (hDir2, n), .0), shininess);

	const vec3 diffuseColor3 = vec3 (.3, .3, .9);
    const vec3 specularColor3 = vec3 (.8, .9, .7);
    vec3 lPos3 = vec3 (sin (t), .5*cos(t), -1.);
    float lDist3 = distance (lPos3, p);
    vec3 lDir3 = normalize (lPos3 - p);
    vec3 hDir3 = normalize (ro + lDir3);
    float diffuse3 = max (dot (n, lDir3), .0)*(1. / lDist3)*diffuseStrength;
    float specular3 = pow (max (dot (hDir3, n), .0), shininess);

    vec3 col = ambient +
			   diffuse * diffuseColor + specular * specularColor +
			   diffuse2 * diffuseColor2 + specular2 * specularColor2 +
			   diffuse3 * diffuseColor3 + specular3 * specularColor3;
    return col;
}

vec3 camera (in vec2 uv, in vec3 ro, in vec3 aim, in float zoom)
{
    vec3 camForward = normalize (vec3 (aim - ro));
    const vec3 worldUp = vec3 (.0, 1., .0);
    vec3 camRight = normalize (cross (worldUp, camForward));
    vec3 camUp = normalize (cross (camForward, camRight));
    vec3 camCenter = ro + camForward * zoom;
    
    return normalize (camCenter + uv.x * camRight + uv.y * camUp - ro);
}

void main ()
{
    vec2 uv = fragCoord.xy;
	vec2 uvRaw = uv;
	uvRaw *= (iResolution.x / iResolution.y, 1.);
    uv = uv *2. - 1.;
    uv.x *= iResolution.x / iResolution.y;

    // set up "camera", view origin (ro) and view direction (rd)
    float angle = radians (300. + 55. * iTime);
    const float dist = 1.5;
    vec3 ro = vec3 (dist * cos (angle), cos (iTime), dist * sin (angle));
    const vec3 aim = vec3 (.0);
    const float zoom = 2.5;
    vec3 rd = camera (uv, ro, aim, zoom);

    // primary-/view-ray
    float d = raymarch (ro, rd);
    vec3 p = ro + d * rd;
    vec3 n = normal (p, d*EPSILON);
    vec3 col = shade (ro, rd, d);
    col = mix (col, vec3 (.95, .85, .7), pow (1. - 1. / d, 10.));

    // secondary-/reflection-ray
    vec3 rd2 = normalize (reflect (rd, n));
    float d2 = raymarch (p + n*EPSILON, rd2);
    vec3 p2 = p + d2 * rd2;
    vec3 n2 = normal (p2, EPSILON);
    vec3 col2 = shade (p, rd2, d2);
    col += (.1 + .05*(.5 + .5 * cos (5.*iTime))) * col2;

    // gamma-correction, tint, vingette
    col = .2 * col + .8 * sqrt (col);
    col *= vec3 (.7, .8, .9);
    col *= .2 + .8 * pow (16. * uvRaw.x * uvRaw.y * (1. - uvRaw.x) * (1. - uvRaw.y), .3);

    fragColor = vec4(col, 1.);
}
