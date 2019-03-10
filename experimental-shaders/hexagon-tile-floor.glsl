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

const int MAX_ITER    = 96;
const float STEP_SIZE = .5;
const float EPSILON   = .0005;
const float PI = 3.14159265359;

mat2 r2d (in float a) { float c = cos(a); float s = sin (a); return mat2 (vec2 (c, s), vec2 (-s, c));}

struct Result {
	float d;
	int id;
};

vec2 noise2d (in vec2 p)
{
    vec3 v = fract (p.xyx*vec3(123.34, 234.34, 345.65));
    v += dot (v, v + 34.45);
    return fract (vec2 (v.x*v.y, v.y*v.z));
}

// basic sdf toolbox
vec2 opRepeat2 (inout vec2 p,in vec2 s) {vec2 h=.5*s; vec2 c=floor((p+h)/s); p=mod(p+h,s)-h; return c;}


float sdHexPrism (in vec3 p, in vec2 h)
{
	vec3 q = abs (p);
	return max (q.z - h.y, max ((q.x * .866025 + q.y * .5), q.y) - h.x);
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

float GeometrySchlickGGX (in float NdotV, in float roughness)
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

float opCombine (in float d1, in float d2, in float r)
{
    float h = clamp (.5 + .5 * (d2 - d1) / r, .0, 1.);
    return mix (d2, d1, h) - r * h * (1. - h);
}

vec2 mapToScreen (in vec2 p)
{
    vec2 res = p;
    res = res * 2. - 1.;
    res.x *= iResolution.x / iResolution.y;
    
    return res;
}

// ray-marching stuff
Result scene (in vec3 p)
{
	p.x += iTime;
	float radius = .195;
	float height = .4;
	vec3 offset = vec3 (.375, .0, .25);
	vec3 hexCenter1 = p;
	vec3 hexCenter2 = p + offset;
	vec2 repeatPattern = vec2(.75, .475);
	hexCenter1.yz *= r2d (radians (90.));
	hexCenter2.yz *= r2d (radians (90.));
    vec2 cellIndex1 = opRepeat2 (hexCenter1.xy, repeatPattern);
    vec2 cellIndex2 = opRepeat2 (hexCenter2.xy, repeatPattern);
	vec2 size = vec2 (radius, height);
	float variance1 = .1*sin (noise2d (cellIndex1).x*iTime);
	float variance2 = .1*sin (noise2d (cellIndex2).y*iTime);
	float hex1 = sdHexPrism (hexCenter1 + vec3 (.0, .0, variance1), size);
	float hex2 = sdHexPrism (hexCenter2 + vec3 (.0, .0, variance2), size);
	float d = hex1;
	d = min (d, hex2);

    Result res = Result (.0, 0);
	res.d = d;
    res.id = 2;

    return res;
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

vec3 shadePBR (in vec3 ro, in vec3 rd, in float d, in int id)
{
    vec3 p = ro + d * rd;
    vec3 nor = normal (p);

    // "material" hard-coded for the moment
    float mask1 = .5 + .5 * cos (20.* p.x * p.z);
    float mask2 = 0.;
	float mask = (id == 1) ? mask1 : mask2;
    vec3 albedo = vec3 (.45, .3, .65);
    float metallic  = (id == 1) ? .1 : .9;
    float roughness = (id == 1) ? .9 : .1;
    float ao = 1.;

    // lights hard-coded as well atm
    vec3 lightColors[3];
    lightColors[0] = vec3 (.5, .7, .9) * 10.;
    lightColors[1] = vec3 (.9, .7, .5) * 20.;
    lightColors[2] = vec3 (.7, .9, .5) * 30.;

    vec3 lightPositions[3];
    lightPositions[0] = p + vec3 (.9, .6, -1.5);
    lightPositions[1] = p + vec3 (-.6, .4, -.75);
    lightPositions[2] = p + vec3 (-.6, .5, .75);

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

vec3 camera (in vec2 uv, in vec3 ro, in vec3 aim, in float zoom)
{
    vec3 camForward = normalize (vec3 (aim - ro));
    vec3 worldUp = vec3 (.0, 1., .0);
    vec3 camRight = normalize (cross (camForward, worldUp));
    vec3 camUp = normalize (cross (camRight, camForward));
    vec3 camCenter = normalize (ro + camForward * zoom);

    return normalize ((camCenter + uv.x*camRight + uv.y*camUp) - ro);
}

void main ()
{
    // normalizing and aspect-correction
	vec2 uvRaw = fragCoord.xy;
	vec2 uv = uvRaw;
    uv = uv * 2. - 1.;
    uv.x *= iResolution.x / iResolution.y;

    // set up "camera", view origin (ro) and view direction (rd)
    vec3 ro = vec3 (5.*cos (.3*iTime), 3. + .1*cos (iTime), 5.*sin (.3*iTime));
    vec3 aim = vec3 (.0, .0, .0);
    float zoom = 1.;
    vec3 rd = camera (uv, ro, aim, zoom);

    // do the ray-march...
    Result res = raymarch (ro, rd);
    float fog = 1. / (1. + res.d * res.d * .1);
    vec3 c = shadePBR (ro, rd, res.d, res.id);

    // tonemapping, "gamma-correction", tint, vignette
	c *= fog;
	c = c / (1. + c);
    c = .2 * c + .8 * sqrt (c);
    c *= vec3 (.9, .8, .7);
    c *= .2 + .8*pow(16.*uvRaw.x*uvRaw.y*(1. - uvRaw.x)*(1. - uvRaw.y), .3);

	fragColor = vec4(c, 1.);
}

