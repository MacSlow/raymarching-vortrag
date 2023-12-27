#version 420
uniform vec3 iResolution;
uniform float iTime;
uniform vec3 iChannelResolution0;
uniform vec3 iChannelResolution1;
uniform vec3 iChannelResolution2;
uniform vec3 iChannelResolution3;
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
const float STEP_SIZE = .65;
const float EPSILON   = .00025;
const float PI = 3.14159265359;

float saturate (in float v)
{
	return clamp (v, .0, 1.);
}

mat2 r2d (in float degree)
{
	float rad = radians (degree);
	float c = cos (rad);
	float s = sin (rad);
	return mat2 (vec2 (c, s), vec2 (-s, c));
}

struct Result {
	float d;
	int id;
};

// basic sdf toolbox
vec3 opRepeat (in vec3 p, in vec3 size) {return mod (p, 2. * size) - size;}
float sdTorus (in vec3 p, in vec2 t) { vec2 q = vec2 (length (p.xz) - t.x, p.y); return length (q) - t.y; }
float udRoundBox (in vec3 p, in vec3 size, in float r) { return length (max (abs (p) - (size - r), .0)) - r; }
float sdSphere (in vec3 p, float r) { return length (p) - r; }
vec2 opRepeat2 (inout vec2 p,in vec2 s) {vec2 h=.5*s; vec2 c=floor((p+h)/s); p=mod(p+h,s)-h; return c;}

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
	float offset = 1.;
	offset += .1*(.5 + .5*cos(p.x*p.x + p.z*p.z + 5.*iTime));
    float ground = p.y + offset;

	offset += .1*(cos(4.*p.x)*sin(4.*p.z));

	p.yz *= r2d (90.);

	vec3 torusCenter1 = p;
	torusCenter1 -= vec3 (.0, .0, .0);
	float torus1 = sdTorus (torusCenter1, vec2 (.8, .1));

	vec3 torusCenter2 = torusCenter1;
	torusCenter2.xz *= r2d (45.*iTime);
	torusCenter2 -= vec3 (.3, .0, .0);
	torusCenter2.yz *= r2d (135.*iTime);
	float torus2 = sdTorus (torusCenter2, vec2 (.4, .05));

	vec3 torusCenter3 = torusCenter2;
	torusCenter3.xz *= r2d (-75.*iTime);
	torusCenter3 -= vec3 (-.15, .0, .0);
	torusCenter3.yz *= r2d (135*iTime);
	float torus3 = sdTorus (torusCenter3, vec2 (.2, .025));

	vec3 torusCenter4 = torusCenter3;
	torusCenter4.xz *= r2d (150.*iTime);
	torusCenter4 -=  vec3 (-.075, .0, .0);
	torusCenter4.yz *= r2d (135.*iTime);
	float torus4 = sdTorus (torusCenter4, vec2 (.1, .025));

	vec3 sphereCenter = torusCenter4;
	float sphere = sdSphere (sphereCenter, .035);

    float d = opCombine (torus1, torus2, 0.2);
	d = opCombine (d, torus3, 0.1);
	d = opCombine (d, torus4, 0.05);
	d = min (d, sphere);

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

float xor (in float a, in float b)
{
	return a*(1. - b) + b*(1. - a);
}

vec3 floorPattern (in vec3 p)
{
    vec2 uv = p.xz;
	uv *= r2d (5.*iTime);

	vec2 grid = fract (uv) - .5;
	vec2 cell = uv - grid;
    vec3 col = vec3 (.0);

	float c = .0;
	for (float y = -1.; y <= 1.; y++) {
		for (float x = -1.; x <= 1.; x++) {
			vec2 offset = vec2 (x, y);

			float spot = length (grid - offset);
			float distanceOverCells = length (cell + offset)*mix (.1, .4, .5 + .5*sin(iTime));

			float radius = mix (.1, 1.5, (.5 + .5*sin (distanceOverCells - 4.*iTime)));
			c = xor (c, smoothstep (radius, radius*.75, spot));
		}
	}
	vec3 a = vec3 (.0);
	vec3 b = vec3 (1.);
	col += vec3 (mix (a, b, mod (c, 2.)));
    return col;
}

vec3 shadePBR (in vec3 ro, in vec3 rd, in float d, in int id)
{
    vec3 p = ro + d * rd;
    vec3 nor = normal (p);

    // "material" hard-coded for the moment
    vec3 albedo1 = floorPattern (4.*p);
    vec3 albedo2 = vec3 (.95, .95, .25);
    vec3 albedo = (id == 1) ? albedo1 : albedo2;
    float metallic  = (id == 1) ? .2 : .75;
    float roughness = (id == 1) ? .8 : .25;
    float ao = 1.;

    // lights hard-coded as well atm
    vec3 lightColors[2];
    lightColors[0] = vec3 (.85, .85, .9) * 20.;
    lightColors[1] = vec3 (.9, .85, .85) * 20.;

    vec3 lightPositions[2];
    vec3 lightPositionsOffset[2];
    lightPositionsOffset[0] = vec3 (.5, .4, .5);
    lightPositionsOffset[1] = vec3 (-.5, .3, -.5);
    lightPositionsOffset[0].xz *= r2d (45.*iTime);
    lightPositionsOffset[1].xz *= r2d (-80.); 
    lightPositions[0] = p + lightPositionsOffset[0];
    lightPositions[1] = p + lightPositionsOffset[1];

	vec3 N = normalize (nor);
    vec3 V = normalize (ro - p);

    vec3 F0 = vec3 (0.04); 
    F0 = mix (F0, albedo, metallic);
    vec3 kD = vec3(.0);
		           
    // reflectance equation
    vec3 Lo = vec3 (.0);
    for(int i = 0; i < 2; ++i) 
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
	vec3 offset = 5.*vec3 (1., .0, 1.);
	float angle = 130. * (iMouse.x / iResolution.x * 2. - 1.);
	offset.xz *= r2d (130. + angle);
	float height = 2.*(iMouse.y / iResolution.y * 2. - 1.);
    vec3 ro = vec3 (0.0, 4.0 - height, 0.0) + offset;
    vec3 aim = vec3 (0.0, 2.0, 0.0);
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
    c *= vec3 (.9, .85, .8);
    c *= .2 + .8*pow(16.*uvRaw.x*uvRaw.y*(1. - uvRaw.x)*(1. - uvRaw.y), .3);

	fragColor = vec4(c, 1.);
}

