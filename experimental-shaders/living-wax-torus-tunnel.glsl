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

#define MAX_STEPS 80
#define STEP_BIAS .5
#define EPSILON .0002
#define PI 3.14159265359

struct Result {
	float d;
	int id;
};

float hashn (vec3 p, float t)
{
    p = fract (p*.3183099 + .1);
    p *= 17.;
    return max (fract (p.x*p.y*p.z*(p.x + p.y + p.z)), 1. - t);
}

float noise (in vec3 x, in float t)
{
    vec3 p = floor (x);
    vec3 f = fract (x);
    f = f*f*(3. - 2.*f);
	
    return mix(mix(mix( hashn(p+vec3(0,0,0), t), 
                        hashn(p+vec3(1,0,0), t),f.x),
                   mix( hashn(p+vec3(0,1,0), t), 
                        hashn(p+vec3(1,1,0), t),f.x),f.y),
               mix(mix( hashn(p+vec3(0,0,1), t), 
                        hashn(p+vec3(1,0,1), t),f.x),
                   mix( hashn(p+vec3(0,1,1), t), 
                        hashn(p+vec3(1,1,1), t),f.x),f.y),f.z);
}

float smax (in float a, in float b, in float k)
{    
   float f = max(0., 1. - abs(b - a)/k);
   return max(a, b) + k*.25*f*f;
}

float sdSphere (in vec3 p, in float radius)
{
    return length (p) - radius;
}

float sdTorus (in vec3 p, in vec2 t)
{
	vec2 q = vec2 (length (p.xz) - t.x, p.y);
	return length (q) - t.y;
}

float sdBox (in vec3 p, in vec3 b)
{
	vec3 d = abs (p) - b;
	return min (max (d.x, max (d.y, d.z)), .0) + length (max (d, .0));
}

mat2 r2d (in float degree)
{
    float rad = radians (degree);
    float c = cos (rad);
    float s = sin (rad);
	return mat2 (c, s, -s, c);
}

vec3 opTwist (in vec3 p)
{
    mat2  m = r2d (p.y*p.x*p.z);
    vec3  q = vec3 (m*p.xz, p.y);
    return q;
}

vec3 sunCenter = vec3 (.0f);

float displace (vec3 p)
{
	float result = 1.f;
	float factor = 6.f + 4.f*cos (2.f*iTime);
	result = .375f*sin (factor * p.x)*cos (factor*p.y)*sin (factor*p.z);
	return result;
}

Result scene (in vec3 p, in float t)
{
	p.xy *= r2d (25.f*iTime);
	p.y += 1.25f*cos (iTime);

    vec3 noiseCenter = p;
    noiseCenter.z -= 2.f; 
    vec3 boxCenter = noiseCenter;
    vec3 ballCenter = noiseCenter; 
    noiseCenter += .05f*opTwist (.5f*p);
    noiseCenter.xy *= r2d (2.f*t);
    noiseCenter.yz *= r2d (-5.f*t);
    noiseCenter.zx *= r2d (10.f*t);

    float variation = 6.f + 3.f*(.5f + .5f*cos (14.f*t));
    vec3 offset = vec3 (-5.f, -1.f, 1.f)*t*.5f;
	float f = 3.f + 2.5f*(.5f + .5f*cos (2.5f*t));
    float structure = noise (f*noiseCenter + offset, variation) -
							 .65f + (sin (t) + 1.f)*.05f;

	vec2 torusSizes = vec2 (2.25f, 1.5f);
	float torus = sdTorus (ballCenter - vec3 (1.f*sin(iTime), .0f, .0f),
						   torusSizes);
    float box = sdBox (boxCenter, vec3 (8.f));

	float r1 = 1.5f;
	float x = 1.f*sin(iTime) + r1*cos (iTime);
	float y = .7f*cos (3.f*iTime);
	float z = .0f + r1*sin (iTime);
	sunCenter = ballCenter - vec3 (x, y, z);

	sunCenter.xz *= r2d (74.f*iTime);
	sunCenter.yx *= r2d (95.f*iTime);
	sunCenter.zy *= r2d (-85.f*iTime);
	float r2 = .35f;
	float a = sdSphere (sunCenter, r2);
	float b = displace (sunCenter);
	float sun = a + b;

    float ball = sdSphere (ballCenter, 1.5f + .5f*(.5f + .5f*cos (t)));
    ball = min (ball, torus);
    structure = smax (-torus, structure*.25f, .25f);
	float bone = smax (box, structure, .2f);

	float d = min (sun, bone);
	int id = (d >= sun) ? 0 : 1;
	Result res = Result (d, id);

    return res;
}

float distriGGX (in vec3 N, in vec3 H, in float roughness)
{
    float a2     = roughness * roughness;
    float NdotH  = max (dot (N, H), .0);
    float NdotH2 = NdotH * NdotH;

    float nom    = a2;
    float denom  = (NdotH2 * (a2 - 1.) + 1.);
    denom        = PI * denom * denom;

    return nom / denom;
}

float geomSchlickGGX (in float NdotV, in float roughness)
{
    float nom   = NdotV;
    float denom = NdotV * (1. - roughness) + roughness;

    return nom / denom;
}

float geomSmith (in vec3 N, in vec3 V, in vec3 L, in float roughness)
{
    float NdotV = max (dot (N, V), .0);
    float NdotL = max (dot (N, L), .0);
    float ggx1 = geomSchlickGGX (NdotV, roughness);
    float ggx2 = geomSchlickGGX (NdotL, roughness);

    return ggx1*ggx2;
}

vec3 fresnelSchlick (in float cosTheta, in vec3 F0, float roughness)
{
	return F0 + (max (F0, vec3 (1.f - roughness)) - F0)*
		   pow (1.f - cosTheta, 5.f);
}

vec3 normal (vec3 p, float epsilon, in float t)
{
    float d = scene (p, t).d;
    vec2 e = vec2 (epsilon, .0f);
    return normalize (vec3 (scene(p + e.xyy, t).d,
                            scene(p + e.yxy, t).d,
                            scene(p + e.yyx, t).d) - d); 
}

Result trace (in vec3 o, in vec3 r)
{
	Result res = Result (.0f, 0);
    float t = .0f;
    for (int i = 0; i < MAX_STEPS; ++i) {
		vec3 p = o + r*t;
		res = scene (p, iTime);
		if (abs (res.d) < EPSILON*(1.f + .125*res.d)) break;
		t += res.d*STEP_BIAS;
    }
	res.d = t;

    return res;
}

float shadow (in vec3 p, in vec3 n, in vec3 lPos)
{
	float distanceToLight = distance (lPos, p);
	float distanceToObject = trace (p + .01f*n,
									normalize (lPos - p)).d;
	bool isShadowed = distanceToObject < distanceToLight;
	return isShadowed ? .1 : 1.;
}

vec3 shade (in vec3 ro, in vec3 rd, in float d, in float t, in int id)
{
    vec3 p = ro + d * rd;
    vec3 nor = normal (p, d*EPSILON, t);

    // "material" hard-coded for the moment
    vec3 albedo = vec3 (.275f);
    float metallic = .15f;
    float roughness = .125f;

	if (id == 0) {
    	albedo = vec3 (.1f, .2f, .4f);
    	metallic = .02f;
    	roughness = .1f;
	}

    // lights hard-coded as well atm
    vec3 lightColors[2];
    lightColors[0] = .5f*vec3 (.5f, .2f, .1f);
    lightColors[1] = .5f*vec3 (.1f, .3f, .5f);

    vec3 lightPositions[2];
    float c = cos (t);
    float s = sin (t);
    lightPositions[0] = vec3 (-.5*c, -.5*s, -.1);
    lightPositions[1] = vec3 (.5*c, .5*s, 1.);

	vec3 N = normalize (nor);
    vec3 V = normalize (ro - p);

    vec3 F0 = vec3 (.04);
    F0 = mix (F0, albedo, metallic);
    vec3 kD = vec3 (.0);
	           
    // reflectance equation
    vec3 Lo = vec3 (.0f);
    for (int i = 0; i < 2; ++i) 
    {
        // calculate per-light radiance
        vec3 L = normalize (lightPositions[i] - p);
        vec3 H = normalize (V + L);
        float dist = distance (p, lightPositions[i]);
        float attenuation = 2.f/(dist*dist);
        vec3 radiance = lightColors[i]*attenuation;
        
        // cook-torrance brdf
        float aDirect = pow (roughness + 1.f, 2.f);
        float aIBL =  roughness * roughness;
        float NDF = distriGGX (N, H, roughness);
        float G = geomSmith (N, V, L, roughness);
        vec3 F = fresnelSchlick (max (dot (H, V), .0f), F0, roughness);

        vec3 kS = F;
        kD = vec3 (1.f) - kS;
        kD *= 1.f - metallic;
        
        vec3 nominator = NDF * G * F;
        float denominator = 4.f*max (dot (N, V), .0f)*max (dot (N, L), .0f);
        vec3 specular = nominator / max (denominator, .001f);

        // add to outgoing radiance Lo
        float NdotL = max (dot (N, L), .0f);
        Lo += (kD*albedo/PI + specular)*radiance*NdotL;
    }

    vec3 ambient = kD * albedo;

    return ambient + Lo;
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
    // normalize and aspect-correct
    vec2 uv = fragCoord;
    uv = uv * 2. - 1.;
    uv.x *= iResolution.x/iResolution.y;

    // set up view-ray/"camera"
    vec3 ro = vec3 (.0f, .0f, -.95f);
	vec3 rd = camera (ro, vec3 (.0f), 1.05f, uv);

    // determine pixel-color
    Result res = trace (ro, rd);
    vec3 color = shade (ro, rd, res.d, iTime, res.id);

    // tone-map, gamma-correct, "fog"
    color = color / (1.f + color);
    color = pow (color, vec3 (1.f/2.2f));
    color *= 2.15f - res.d*.3125f;

    fragColor = vec4 (color, 1.);
}
