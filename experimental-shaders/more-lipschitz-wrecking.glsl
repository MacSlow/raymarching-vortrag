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

const int MAX_ITER = 80;
const float EPSILON = .0001;
const float STEP_SIZE = .5;
const int AA_SIZE = 2;

float opBend (inout vec3 p, float deg)
{
    float rad = radians (deg);
    float cy = cos (rad * p.y);
    float sy = sin (rad * p.y);
    mat2  my = mat2 (cy, -sy, sy, cy);
    float cx = cos (rad * p.x);
    float sx = sin (rad * p.x);
    mat2  mx = mat2 (cx, -sx, sx, cx);
    p = mix (vec3 (mx* p.zx, p.y), vec3 (my * p.xy, p.z), .15*cos (p.x));

    return .0;
}

float displace (vec3 p)
{
    float result = 1.;
    float factor = 6. + 4.*cos (2.*iTime);
	result = .375 * sin (factor * p.x) * cos (factor * p.y) * sin (factor * p.z);

    return result;
}

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
	// well, kind of self explanatory
    float ground = sdPlane (p, -2.);

	// the weird distorted thing in the middle of the scene
    vec3 p2 = p;
    opBend (p2, 45. * cos (.25*iTime));
    p2.zx *= r2d (50.*iTime);
    p2.xy *= r2d (-75.*iTime);
    float dt = sdSphere (p2, .5);
    float dp = displace (p2);
    float ball2 = dt + dp;

	// the structure rotating around the scene
    p.xz *= r2d (20.*iTime);
    p.zy *= r2d (-30.*iTime);
    float x = p.x*.125;
    float y = p.y*.125;
    float z = p.z*.125;
    float fourthOrderChmutovBanchoffSurface = 3. + 8.*(x*x*x*x + y*y*y*y + z*z*z*z) - 8.*(x*x + y*y + z*z);
    fourthOrderChmutovBanchoffSurface *= .75;
    ground = opCombine (ground, ball2, 2.5);
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

	return isShadowed ? .3 : 1.;
}

// rusty old blinn/phong shading model
vec3 shade (in vec3 p, in vec3 rd, in vec3 n)
{
	// attributes of first light
    vec3 lightPosition1 = vec3 (1.);
    lightPosition1.xz *= r2d (60.*iTime);
    vec3 l1 = normalize (lightPosition1 - p);
    float d1 = distance (p, lightPosition1);
    float lightIntensity1 = 5.;
	float att1 = 2./(d1*d1);
    vec3 lightColor1 = vec3 (.9, .8, .7);

	// attributes of second light
 	vec3 lightPosition2 = vec3 (1., 1.*cos (2.*iTime), 1.);
    lightPosition2.xz *= r2d (20.*iTime);
    vec3 l2 = normalize (lightPosition2 - p);
    float d2 = distance (p, lightPosition2);
    float lightIntensity2 = 3.;
	float att2 = 2./(d2*d2);
    vec3 lightColor2 = vec3 (.7, .8, .9);

	// diffuse term parts
    vec3 diffuseColor1 = max (dot (n, l1), .0) * lightColor1 * lightIntensity1;
    vec3 diffuseColor2 = max (dot (n, l2), .0) * lightColor2 * lightIntensity2;

	// specular coefficient
	float shiny = 60.;
	vec3 h1 = normalize (-rd + l1);
	vec3 h2 = normalize (-rd + l2);
	float sp1 = pow (max(.0, dot (h1, l1)), shiny);
	float sp2 = pow (max(.0, dot (h2, l2)), shiny);

	// there is only one 'material'
    vec3 matertialColor = vec3 (.4, .7, .6);

    return att1*shadow (p, lightPosition1) * (matertialColor*diffuseColor1 + sp1*vec3 (1.))+
           att2*shadow (p, lightPosition2) * (matertialColor*diffuseColor2 + sp2*vec3 (1.));
}

vec3 camera (in vec2 uv, in vec3 ro, in vec3 aim, in float zoom)
{
	vec3 f = normalize (aim - ro);
	vec3 wu = vec3 (.0, 1., .0);
	vec3 r = normalize (cross (wu, f));
	vec3 u = normalize (cross (f, r));
	vec3 c = ro + f*zoom;
	return normalize (c + r*uv.x + u*uv.y - ro);
}

void main ()
{
	// orbit camera preparation
    float dist2cam = 3.;
    float azimuthAngle = ((iMouse.x / iResolution.x) * 2. - 1.) * 179.;
    float elevationAngle = ((iMouse.y / iResolution.y) * 2. - 1.) * -40.;
    float x = dist2cam*cos (radians (azimuthAngle));
    float y = dist2cam*sin (radians (elevationAngle));
    float z = dist2cam*sin (radians (azimuthAngle));

	// stuff
    vec2 uv = fragCoord.xy;
    vec3 ro = vec3 (x, y, z);
	float fog = .0;
	float d = .0;
	vec3 col = vec3 (.0);
	vec3 aim = vec3 (.0);
	float zoom = 1.7;

	// walk over AA-grid
	for (int x = 0; x < AA_SIZE; ++x) {
		for (int y = 0; y < AA_SIZE; ++y) {

			// anti-alias offset
			vec2 pixelOffset = vec2 (float (x), float (y))/float  (AA_SIZE);

			// normalize and aspect-correct UVs
			vec2 uv = (fragCoord.xy + pixelOffset/iResolution.xy);
    		uv = uv*2. - 1.;
            uv.x *= iResolution.x/iResolution.y;

    		// create viewray
    		vec3 rd = camera (uv, ro, aim, zoom);

			// primary/view ray
			d = raymarch (ro, rd);
			fog = 1. / (1. + d*d*.02);
			vec3 p = ro + d*rd;
			vec3 n = normal (p, d*EPSILON);
			vec3 ctmp = shade (p, rd, n);
			ctmp *= fog;

			// secondary/reflection ray
			vec3 refr = normalize (reflect (rd, n));
			float refd = raymarch (p + .0005*n, refr);
			vec3 refp = p + refd * refr;
			vec3 refn = normal (refp, EPSILON);
			col += ctmp + .05*shade (refp, refr, refn);
		}
	}
	col /= float (AA_SIZE*AA_SIZE);

    // tint, vignette, tone-map, gamma-correct
	col = mix (col, vec3 (.2, .35, .7), pow (1. - 1./d, 90.));
	col *= 1. - .5*length (fragCoord.xy*2. - 1.);
    col = col / (1. + col);
    col = pow (col, vec3 (1./2.2));

    fragColor = vec4 (col, 1.);
}
