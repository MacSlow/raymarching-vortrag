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

const int MAX_ITER    = 96;
const float STEP_SIZE = .5;
const float EPSILON   = .0001;
const float PI = 3.14159265359;

mat2 r2d (in float a)
{
	float c = cos(a);
	float s = sin(a);
	return mat2 (c, s, -s, c);
}

struct Result {
	float d;
	vec3 col;
	int id;
};

// basic sdf toolbox
float udRoundBox (vec3 p, vec3 size, float r)
{
	return length (max (abs (p) - (size - r), .0)) - r;
}

float sdSphere (vec3 p, float r)
{
	return length (p) - r;
}

// PBR toolbox
float DistributionGGX (in vec3 N, in vec3 H, in float roughness)
{
    float a2     = roughness * roughness;
    float NdotH  = max (dot (N, H), .0);
    float NdotH2 = NdotH * NdotH;

    float nom    = a2;
    float denom  = (NdotH2 * (a2 - 1.) + 1.);
    denom        = PI * denom * denom;

    return nom / denom;
}

float GeometrySchlickGGX (float NdotV, float roughness)
{
    float nom   = NdotV;
    float denom = NdotV * (1. - roughness) + roughness;

    return nom / denom;
}

float GeometrySmith (in vec3 N, in vec3 V, in vec3 L, in float roughness)
{
    float NdotV = max (dot (N, V), .0);
    float NdotL = max (dot (N, L), .0);
    float ggx1 = GeometrySchlickGGX (NdotV, roughness);
    float ggx2 = GeometrySchlickGGX (NdotL, roughness);

    return ggx1 * ggx2;
}

vec3 fresnelSchlick (in float cosTheta, in vec3 F0, float roughness)
{
	return F0 + (max (F0, vec3(1. - roughness)) - F0) * pow (1. - cosTheta, 5.);
}

vec4 smin (vec4 d1, vec4 d2, float k)
{
	float h = clamp (.5 + .5*(d2.a - d1.a)/k, .0, 1.);
	return mix (d2, d1, h) - h*k*(1. - h);
}

float fOpUnionStairs (float a, float b, float r, float n)
{
    float s = r/n;
    float u = b - r;
    return min(min(a,b), .5*(u + a + abs ((mod (u - a + s, 2.*s)) - s)));
}

float vary (float value, float speed)
{
	return value*(.5 + .5*cos(speed*iTime));
}

vec2 opRepeatPolar (vec2 p, float repetitions)
{
	float r = length (p);
	float angle = degrees (2.*PI/repetitions);
	float degree = degrees (atan (p.y, p.x)) + angle*.5;
	float rad = radians (mod (degree, angle) - angle*.5);
	return vec2 (cos (rad), sin(rad))*r;
}

// ray-marching stuff
Result scene (in vec3 p)
{
    float ground = p.y + .35;

	vec3 colA = vec3 (1. - vary (1., .75),
					  1. - vary (1., .5),
					  1. - vary (1., .25));

	vec2 size = vec2 (3. + 2.*(.5 + .5*cos(iTime)));
	p.xz = mod (p.xz + .5*size, size) - .5*size;
	vec3 sphereCenter = p;
	float offset = .75 + .75*(.5 + .5*cos(iTime));
	sphereCenter.xz = opRepeatPolar (sphereCenter.xz, 5.);
	float sphere = sdSphere (sphereCenter, .6);

	sphereCenter -= vec3 (offset, -.5*(.5 + .5*cos(iTime)), .0);
	colA = vec3 (1. - vary (1./6., .25),
				 1. - vary (1./6., .5),
				 1. - vary (1./6., .75));
	float stairs = 1. + 4.*(.5 + .5*cos(iTime));
	sphere = fOpUnionStairs (sphere,
							 sdSphere (sphereCenter, .6),
							 .5,
							 stairs);

	vec3 colB = vec3 (vary (.5, 2.), vary (1., 1.), vary (.25, 2.));
    vec4 foo = smin (vec4 (colB, ground), vec4 (colA, sphere), .5);
    float d = foo.a;
	vec3 col = foo.rgb;

    Result res = Result (.0, vec3 (.0), 0);
	res.d = d;
    res.id = (d == ground) ? 1 : 2;
    res.col = col;
    return res;
}

Result raymarch (in vec3 ro, in vec3 rd)
{
    Result res = Result (.0, vec3 (.0), 0);

    for (int i = 0; i < MAX_ITER; i++)
    {
        vec3 p = ro + res.d * rd;
        Result tmp = scene (p);
        if (abs (tmp.d) < EPSILON*(1. + .125*tmp.d)) return res;
        res.d += tmp.d * STEP_SIZE;
        res.id = tmp.id;
		res.col = tmp.col;
    }

    return res;
}

vec3 normal (in vec3 p)
{
    vec2 e = vec2(.0001, .0);
    float d = scene (p).d;
    vec3 n = vec3 (scene (p + e.xyy).d - d,
                   scene (p + e.yxy).d - d,
                   scene (p + e.yyx).d - d);
    return normalize(n);
}

float shadow (in vec3 ro, in vec3 rd)
{
    float result = 1.;
    float t = .1;
    for (int i = 0; i < MAX_ITER; i++) {
        float h = scene (ro + t * rd).d;
        if (h < 0.00001) return .0;
        result = min (result, 8. * h/t);
        t += h;
    }

    return result;
}

float ao (vec3 p, vec3 n, float d) {
    return clamp (scene (p + d*n).d/d, .0, 1.);
}

vec3 shadePBR (vec3 ro, vec3 rd, float d, int id, vec3 col)
{
    vec3 p = ro + d*rd;
    vec3 nor = normal (p);

    // "material" hard-coded for the moment
	float a = texture (iChannel1, p.xz).r;
	vec2 offset = 2.*vec2 (cos(a), sin(a));
	vec2 uv = p.xz + offset;
	vec3 col2 = texture (iChannel0, uv).rgb;
	//col2 = mix (col, col2, .95);
	nor = normalize (nor + .9*col2);
    vec3 albedo = col2;
    float metallic  = .6;
    float roughness = .4;
    float ao = ao (p, nor, .25);

    // lights hard-coded as well atm
    vec3 lightColors[3];
    lightColors[0] = vec3 (.7, .8, .9)*20.;
    lightColors[1] = vec3 (.9, .8, .7)*20.;
    lightColors[2] = vec3 (.9, .3, .2)*20.;

    vec3 lightPositions[3];
    lightPositions[0] = p + vec3 (.5, .75, -1.5);
    lightPositions[1] = p + vec3 (-.5, .25, 1.);
    lightPositions[2] = p + vec3 (-.1, 1., .1);

	vec3 N = normalize (nor);
    vec3 V = normalize (ro - p);

    vec3 F0 = vec3 (0.04); 
    F0 = mix (F0, albedo, metallic);
    vec3 kD = vec3(.0);
		           
    // reflectance equation
    vec3 Lo = vec3 (.0);
    for(int i = 0; i < 3; ++i) 
    {
        // calculate per-light radiance
        vec3 L = normalize(lightPositions[i] - p);
        vec3 H = normalize(V + L);
        float distance    = length(lightPositions[i] - p);
        float attenuation = 1. / (distance * distance);
        vec3 radiance     = lightColors[i] * attenuation;
	        
        // cook-torrance brdf
        float aDirect = .125 * pow (roughness + 1., 2.);
        float aIBL = .5 * roughness * roughness;
        float NDF = DistributionGGX(N, H, roughness);        
        float G   = GeometrySmith(N, V, L, roughness);      
        vec3 F    = fresnelSchlick(max(dot(H, V), 0.0), F0, roughness);
	        
        vec3 kS = F;
        kD = vec3(1.) - kS;
        kD *= 1. - metallic;	  
	        
        vec3 nominator    = NDF * G * F;
        float denominator = 4. * max(dot(N, V), 0.0) * max(dot(N, L), 0.0);
        vec3 specular     = nominator / max(denominator, .001);  

        // add to outgoing radiance Lo
        float NdotL = max(dot(N, L), 0.0);                
        Lo += (kD * albedo / PI + specular) * radiance * NdotL; 
	    Lo *= shadow (p, L);
    }

    vec3 ambient = (kD * albedo) * ao;

    return ambient + Lo;
}

vec3 camera (vec2 uv, vec3 ro, vec3 aim, float zoom)
{
    vec3 f = normalize (vec3 (aim - ro));
    vec3 wu = vec3 (.0, 1., .0);
	wu.xy *= r2d (radians (50.*cos(iTime)));
    vec3 r = normalize (cross (wu, f));
    vec3 u = normalize (cross (f, r));
    vec3 c = ro + f*zoom;

    return normalize (c + uv.x*r + uv.y*u - ro);
}

void main ()
{
	vec2 uvRaw = fragCoord.xy;
	vec2 uv = uvRaw;
    uv = uv*2. - 1.;
    uv.x *= iResolution.x/iResolution.y;

    vec3 ro = vec3 (-2.*cos(iTime), 2., -2.*sin(iTime));
    vec3 aim = vec3 (.0);
    float zoom = 2.;
    vec3 rd = camera (uv, ro, aim, zoom);

    Result res = raymarch (ro, rd);
    float fog = 1. / (1. + res.d * res.d * .1);
    vec3 c = shadePBR (ro, rd, res.d, res.id, res.col);

	c *= fog;
	c = c / (1. + c);
    c = .2 * c + .8 * sqrt (c);
    c *= vec3 (.9, .8, .7);
    c *= .2 + .8*pow(16.*uvRaw.x*uvRaw.y*(1. - uvRaw.x)*(1. - uvRaw.y), .3);

	fragColor = vec4(c, 1.);
}

