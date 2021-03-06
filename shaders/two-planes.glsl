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

const int MAX_ITER    = 48;
const float STEP_SIZE = .95;
const float EPSILON   = .001;
const float PI = 3.14159265359;

float saturate (in float v) { return clamp (v, .0, 1.); }
mat2 r2d (in float a)
{
	float c = cos(a);
	float s = sin (a);
	return mat2 (vec2 (c, s), vec2 (-s, c));
}

float opUnion (in float d1, in float d2) { return min (d1, d2); }
float opSubtract (in float d1, in float d2){ return max (-d1, d2); }
float opIntersect (in float d1, in float d2) { return max (d1, d2); }

float pModInterval1(inout float p, float size, float start, float stop) {
	float halfsize = size*0.5;
	float c = floor((p + halfsize)/size);
	p = mod(p+halfsize, size) - halfsize;
	if (c > stop) { //yes, this might not be the best thing numerically.
		p += size*(c - stop);
		c = stop;
	}
	if (c <start) {
		p += size*(c - start);
		c = start;
	}
	return c;
}

struct Result {
	float d;
	int id;
};

// basic sdf toolbox
float udRoundBox (in vec3 p, in vec3 size, in float r)
{
	return length (max (abs (p) - (size - r), .0)) - r;
}

float sdBox (in vec3 p, in vec3 size, in float r)
{
	vec3 d = abs(p) - size;
	return min (max (d.x, max (d.y,d.z)), .0) + length (max (d, .0)) - r;
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

float dd = .0;

// ray-marching stuff
Result scene (in vec3 p)
{
	float offset = 2.5;
    float ground = p.y + offset;
    float wall = p.z - 2.*offset;
    ground = min (-wall, ground);

	vec3 boxCenter = p;
	boxCenter -= vec3 (-1.0, -.25, 1.75);
	boxCenter.xz *= r2d (-iTime);
	boxCenter.xy *= r2d (-.8*iTime);
	boxCenter.yz *= r2d (1.2*iTime);

    float timeScale = .5 + .5 * cos ((1.125+.125*sin(.1*iTime))*iTime);
	float size = .2 + .7 * timeScale;
	float start = -1.;
	float stop = 1.;
	pModInterval1 (boxCenter.x, size, start, stop);
	pModInterval1 (boxCenter.y, size, start, stop);
	pModInterval1 (boxCenter.z, size, start, stop);
	float box = udRoundBox (boxCenter, vec3 (.2),.2 * timeScale);
    float d = box;

	dd = d;

    Result res = Result (.0, 0);
	res.d = min (d, ground);
    res.id = (res.d == ground) ? 1 : 2;
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

vec3 shade (in vec3 ro, in vec3 rd, in float d, in int id)
{
    vec3 p = ro + d * rd;
    vec3 nor = normal (p);

    // "material" hard-coded for the moment
    float mask1 = .5 + .5 * cos (20.* p.x * p.z);
    float mask2 = 0.;
	float mask = (id == 1) ? mask1 : mask2;
	float f = fract (dd*2.);
    float scale = .5 + .5 * cos (1.75*iTime);
    vec3 albedo1 = vec3 (1. - smoothstep (.025, .0125, f));
    vec3 albedo2 = mix (vec3 (.15, .75, .75), vec3 (.15, .75, .15), scale);
    vec3 albedo3 = mix (albedo2, vec3 (.75, .15, .15), sin(3.141*scale));
    vec3 albedo = (id == 1) ? albedo1 : albedo3;
    float metallic  = (id == 1) ? .1 : .9*scale;
    float roughness = (id == 1) ? .9 : .2 + .7*(1. - scale);
    float ao = 1.;

    // light hard-coded as well atm
    vec3 lightColor = vec3 (.9, .8, .8) * 20.;

    vec3 lightPosition = p + vec3 (-.3, .25, -.5);

	vec3 N = normalize (nor);
    vec3 V = normalize (ro - p);

    vec3 F0 = vec3 (0.04); 
    F0 = mix (F0, albedo, metallic);
    vec3 kD = vec3(.0);
		           
    // reflectance equation
    vec3 Lo = vec3 (.0);

    // calculate per-light radiance
    vec3 L = normalize(lightPosition - p);
    vec3 H = normalize(V + L);
    float distance    = length(lightPosition - p);
    float attenuation = 1. / (distance * distance);
    vec3 radiance     = lightColor * attenuation;
        
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
    vec3 ro = vec3 (2.0, 2.0, -5.0);
    vec3 aim = vec3 (-1.0, 1.5, 0.0);
    float zoom = 1.;
    vec3 rd = camera (uv, ro, aim, zoom);

    // do the ray-march...
    Result res = raymarch (ro, rd);
    float fog = 1. / (1. + res.d * res.d * .1);
    vec3 c = shade (ro, rd, res.d, res.id);

    // tonemapping, "gamma-correction", tint, vignette
	c *= fog;
	c = c / (1. + c);
    c = .2 * c + .8 * sqrt (c);
    c *= vec3 (.9, .8, .7);
    c *= .2 + .8*pow(16.*uvRaw.x*uvRaw.y*(1. - uvRaw.x)*(1. - uvRaw.y), .3);

	fragColor = vec4(c, 1.);
}

