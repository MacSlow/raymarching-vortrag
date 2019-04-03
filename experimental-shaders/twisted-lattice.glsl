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
const float STEP_SIZE = 1.1;
const float EPSILON   = .001;
const float PI = 3.14159265359;
const int AA_SIZE = 3;

mat2 r2d (in float a) {
    float rad = radians (a);
    float c = cos(rad);
    float s = sin (rad);
    return mat2 (c, s, -s, c);
}

struct Result {
	float d;
	int id;
};

// ---- PBR toolbox ------------------------------
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

float smin (in float d1, in float d2, in float r)
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


// ---- Raymarching toolbox ------------------------------

// slightly adapted column-step operator from Mercury's http://mercury.sexy/hg_sdf
float fOpUnionColumns(float a, float b, float r, float n) {
    if ((a < r) && (b < r)) {
        vec2 p = vec2(a, b); 
        float columnradius = r*sqrt(2.)/((n-1.)*2.+sqrt(2.));
        p *= r2d (45.);
        p.x -= sqrt(2.)/2.*r;
        p.x += columnradius*sqrt(2.);
        if (mod(n,2.) == 1.) {
            p.y += columnradius;
        }
        p.y = mod (p.y + columnradius, columnradius*2.)-columnradius;
        float result = length(p) - columnradius;
        result = min(result, p.x);
        result = min(result, a); 
        return min(result, b); 
    } else {
        return min(a, b); 
    }   
}

Result scene (in vec3 p)
{
	p.xz *= r2d (34.*iTime);
	p.xy *= r2d (23.*iTime);
	p.x -= cos (iTime);
	p.z -= sin (iTime);

    // bye-bye Lipschitz continuity... but it looks cool :)
    p.xy *= r2d (3.*p.z);
    p.yz *= r2d (3.*p.x);

	vec3 cylinderCenter = p;
	vec3 size = vec3 (3.5);
	cylinderCenter = mod (cylinderCenter + .5*size, size) - .5*size;
	float cylinder = length (cylinderCenter.xz) - .3;
	cylinder = min (cylinder, length (cylinderCenter.yz) - .3);
	cylinder = min (cylinder, length (cylinderCenter.xy) - .3);
    float d = cylinder;

    vec3 ballCenter = p;
    ballCenter = mod (ballCenter + .5*size, size) - .5*size;

    float ball = length (ballCenter) - .6;

    Result res = Result (.0, 0);
	res.d = fOpUnionColumns (cylinder, ball, .1, 2.);
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
        if (abs (tmp.d) < EPSILON*(1. + .125*tmp.d)) return res;
        res.d += tmp.d * STEP_SIZE;
        res.id = tmp.id;
    }

    return res;
}

vec3 normal (in vec3 p)
{
    vec2 e = vec2(.0001, .0);
    float d = scene (p).d;
    vec3 n = vec3 (scene (p + e.xyy).d,
                   scene (p + e.yxy).d,
                   scene (p + e.yyx).d) - d;
    return normalize(n);
}

float shadow (in vec3 p, in vec3 n, in vec3 ldir, in float ldist)
{
	float d2w = raymarch (p + .01*n, ldir).d;
	return ldist < d2w ? 1. : .1;
}

float ao (vec3 p, vec3 n, float stepsize, int iter, float i) {
	float ao = .0;
	float dist = .0;
	for (int a = 1; a <= iter; ++a) {
		dist = float (a)*stepsize;
		ao += max (.0, (dist - scene (p+n*dist).d)/dist);
	}
	return 1. - ao*i;
}
vec3 shadePBR (in vec3 ro, in vec3 rd, in float d, in int id)
{
    vec3 p = ro + d * rd;
    vec3 nor = normal (p);

    // "material" hard-coded for the moment
    vec3 albedo = vec3 (.65);
    float metallic  = .9;
    float roughness = .1;
    float ao = ao (p, nor, .075, 4, .75);

    // lights hard-coded as well atm
    vec3 lightColors[2];
    lightColors[0] = vec3 (.5, .5, .9) * 60.;
    lightColors[1] = vec3 (.9, .9, .7) * 60.;

    vec3 lightPositions[2];
    lightPositions[0] = vec3 (.5, 2.75, .5);
    lightPositions[1] = vec3 (-.3, .25, -.5);

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
        float ldist = distance (lightPositions[i], p);
        float attenuation = 5./(ldist*ldist);
        vec3 radiance = lightColors[i]*attenuation;
	        
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
	    Lo *= shadow (p, N, L, ldist);
    }

    vec3 ambient = kD*albedo*ao;

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

void main()
{
    // normalizing and aspect-correction
	vec2 uvRaw = fragCoord.xy;

    // set up "camera", view origin (ro) and view direction (rd)
	vec3 ro = vec3 (.0, 1., 2.);
    vec3 aim = vec3 (.0);
    float zoom = 2.;

	float fog = .0;
	Result res;
	vec3 color = vec3 (.0);

	for (int x = 0; x < AA_SIZE; ++x) {
		for (int y = 0; y < AA_SIZE; ++y) {

			// anti-alias offset
			vec2 pixelOffset = vec2 (float (x), float (y))/float  (AA_SIZE) - .5;

			// normalize, aspect-correct and 'bulge' UVs
			vec2 uv = (fragCoord.xy + pixelOffset/iResolution.xy);
			uv = uv * 2. - 1.;
			uv.x *= iResolution.x / iResolution.y;
			uv *= 1. + .5*length (uv);

			// create ray for view direction
    		vec3 rd = camera (uv, ro, aim, zoom);

		    // do the ray-march...
			res = raymarch (ro, rd);
			fog = 1. / (1. + res.d * res.d * .05);
			vec3 ctmp = shadePBR (ro, rd, res.d, res.id);
			ctmp *= fog;

			// do the reflections
			/*if (res.id == 2) {
				vec3 p = ro + res.d*rd;
				vec3 n = normal (p);
				ro = p +.01*n;
				rd = normalize (reflect (rd, n));
				Result res2 = raymarch (ro, rd);
				c += .05*shadePBR (ro, rd, res2.d, res2.id);
			}*/
			color += ctmp;
		}
	}
	color /= float (AA_SIZE*AA_SIZE);

    // distance-mist, tonemapping, tint, vignette, raster-lines, gamma-correction
	color = mix (color, vec3 (.9, .85, .7), pow (1. - 1. / res.d, 30.));
	color = color / (1. + color);
    color *= vec3 (.9, .8, .7);
    color *= .2 + .8*pow(16.*uvRaw.x*uvRaw.y*(1. - uvRaw.x)*(1. - uvRaw.y), .3);
	color *= mix (1., .5, cos (1100.*uvRaw.y));
    color = pow (color, vec3 (1./2.2));

	fragColor = vec4(color, 1.);
}

