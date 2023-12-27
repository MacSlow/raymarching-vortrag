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

const int MAX_ITER = 512;
const float EPSILON = .001;
const float STEP_BIAS = .1;
const int AA_SIZE = 2; // 1: no anti-aliasing (faster), 2 or up: better anti-aliasing (slower)

float sdPlane (in vec3 p, in float height)
{
	return length (p.y - height);
}

mat2 r2d (in float degree)
{
    float rad = radians (degree);
    float c = cos (rad);
	float s = sin (rad);

    return mat2 (c, s, -s, c);
}

// the reason I did this shader... iq's SDF from https://www.shadertoy.com/view/XtjSDK
float sdThingFromIQ (in vec3 p, in float v)
{
    p.xyz += 1.000*sin(  2.0*p.yzx )*v;
    p.xyz += 0.500*sin(  4.0*p.yzx )*v;
    p.xyz += 0.250*sin(  8.0*p.yzx )*v;
    p.xyz += 0.125*sin( 16.0*p.yzx )*v;

    float d = length (p) - 1.5;
	return d*.5;
}

float scene (in vec3 p)
{
    float ground = sdPlane (p, -3.);

    vec3 thingCenter = p;
    float thing = sdThingFromIQ (thingCenter, .5 + .5*cos (iTime));

    return min (ground, thing);
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
        d += t*STEP_BIAS;
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

float ao (in vec3 p, in vec3 n, float stepsize, int iterations, float intensity) {
    float ao = .0; 
    float dist = .0; 
    for (int a = 1; a <= iterations; ++a) {
        dist = float (a)*stepsize;
        ao += max (.0, (dist - scene (p + n*dist))/dist);
    }   
    return 1. - ao*intensity;
}

// rusty old blinn/phong shading model
vec3 shade (in vec3 p, in vec3 rd, in vec3 n)
{
	// attributes of first light
    vec3 lightPosition1 = vec3 (2.);
    lightPosition1.xz *= r2d (60.*iTime);
    vec3 l1 = normalize (lightPosition1 - p);
    float d1 = distance (p, lightPosition1);
    float lightIntensity1 = 5.;
	float att1 = 3./(d1*d1);
    vec3 lightColor1 = vec3 (.95, .7, .6);

	// attributes of second light
 	vec3 lightPosition2 = vec3 (2., 4.75, -1.5);
    lightPosition2.xz *= r2d (20.*iTime);
    vec3 l2 = normalize (lightPosition2 - p);
    float d2 = distance (p, lightPosition2);
    float lightIntensity2 = 8.;
	float att2 = 4./(d2*d2);
    vec3 lightColor2 = vec3 (.2, .4, .9);

	// diffuse term parts
    vec3 diffuseColor1 = max (dot (n, l1), .0) * lightColor1 * lightIntensity1;
    vec3 diffuseColor2 = max (dot (n, l2), .0) * lightColor2 * lightIntensity2;

	// specular coefficient
	float shiny = 100.;
	vec3 h1 = normalize (-rd + l1);
	vec3 h2 = normalize (-rd + l2);
	float sp1 = pow (max(.0, dot (h1, l1)), shiny);
	float sp2 = pow (max(.0, dot (h2, l2)), shiny);

	// there is only one 'material'
    vec3 matertialColor = vec3 (.4, .7, .6);

    float ao = ao (p, n, .2, 6, .175);
    
    return ao*att1*shadow (p, lightPosition1) * (matertialColor*diffuseColor1 + sp1*vec3 (1.))+
           ao*att2*shadow (p, lightPosition2) * (matertialColor*diffuseColor2 + sp2*vec3 (1.));
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

void main()
{
	// orbit camera preparation
    float dist2cam = 5.;
    float azimuthAngle = ((iMouse.x/iResolution.x) * 2. - 1.) * 179.;
    float elevationAngle = 30. + ((iMouse.y/iResolution.y) * 2. - 1.) * -20.;
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

			col += ctmp;
		}
	}
	col /= float (AA_SIZE*AA_SIZE);

    // distance-mist, vignette, tone-map, gamma-correct
	col = mix (col, vec3 (.2, .35, .7), pow (1. - 1./d, 90.));
	col *= 1. - .5*length (fragCoord.xy/iResolution.xy*2. - 1.);
    col = col / (1. + col);
    col = pow (col, vec3 (1./2.2));

    fragColor = vec4 (col, 1.);
}

