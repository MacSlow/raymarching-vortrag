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

const float PI = 3.14159265359;
const bool SHADOW = true;
const int MAX_ITER = 64;
const float EPSILON = .001;
const float STEP_BIAS = .75;
const int AA_SIZE = 2;

float saturate (in float v) { return clamp (v, .0, 1.); }

float distriGGX (in vec3 N, in vec3 H, in float roughness) {
    float a2     = roughness * roughness;
    float NdotH  = max (dot (N, H), .0);
    float NdotH2 = NdotH * NdotH;

    float nom    = a2;
    float denom  = (NdotH2 * (a2 - 1.) + 1.);
    denom        = PI * denom * denom;

    return nom / denom;
}

float geomSchlickGGX (in float NdotV, in float roughness) {
    float nom   = NdotV;
    float denom = NdotV * (1. - roughness) + roughness;

    return nom / denom;
}

float geomSmith (in vec3 N, in vec3 V, in vec3 L, in float roughness) {
    float NdotV = max (dot (N, V), .0);
    float NdotL = max (dot (N, L), .0);
    float ggx1 = geomSchlickGGX (NdotV, roughness);
    float ggx2 = geomSchlickGGX (NdotL, roughness);

    return ggx1 * ggx2;
}

vec3 fresnelSchlick (in float cosTheta, in vec3 F0, float roughness) {
	return F0 + (max (F0, vec3(1. - roughness)) - F0) * pow (1. - cosTheta, 5.);
}

mat2 r2d (in float a) {
	float c = cos (radians (a));
    float s = sin (radians (a));
    return mat2 (vec2(c, s), vec2(-s, c));
}

float opRepeat1 (inout float p, in float size) {
    float hsize = .5 * size;
    float cell = floor ((p + hsize) / size);
    p = mod (p + hsize, size) - hsize;
    return cell;
}

vec2 opRepeat2 (inout vec2 p, in vec2 size) {
    vec2 hsize = .5 * size;
    vec2 cell = floor ((p + hsize) / size);
    p = mod (p + hsize, size) - hsize;
    return cell;
}

float opCombine (in float d1, in float d2, in float r) {
    float h = saturate (.5 + .5 * (d2 - d1) / r);
    return mix (d2, d1, h) - r * h * (1. - h);
}

float sdSphere (in vec3 p, in float r) {
    return length (p) - r;
}

float vmax (in vec3 p) {
    return max (p.x, max (p.y, max (p.z, .0)));
}

float udBox (in vec3 p, in vec3 size) {
    return length (vmax (abs (p) - size));
}

float sdBox (in vec3 p, in vec3 size, in float r)
{
  vec3 d = abs(p) - size;
  return min (max (d.x, max (d.y,d.z)), .0) + length (max (d, .0)) - r;
}

float metaballs (in vec3 p, in float factor) {
    float r1 = factor * .1 + .3 * (.5 + .5 * sin (2. * iTime));
    float r2 = factor * .15 + .2 * (.5 + .5 * sin (3. * iTime));
    float r3 = factor * .2 + .2 * (.5 + .5 * sin (4. * iTime));
    float r4 = factor * .25 + .1 * (.5 + .5 * sin (5. * iTime));

    float t = 2. * iTime;
    vec3 offset1 = vec3 (-.1*cos(t), .1, -.2*sin(t));
    vec3 offset2 = vec3 (.2, .2*cos(t), .3*sin(t));
    vec3 offset3 = vec3 (-.2*cos(t), -.2*sin(t), .3);
    vec3 offset4 = vec3 (.1, -.4*cos(t), .4*sin(t));
    vec3 offset5 = vec3 (.4*cos(t), -.2, .3*sin(t));
    vec3 offset6 = vec3 (-.2*cos(t), -.4, -.4*sin(t));
    vec3 offset7 = vec3 (.3*sin(t), -.6*cos(t), .6);
    vec3 offset8 = vec3 (-.3, .5*sin(t), -.4*cos(t));

    float ball1 = sdSphere (p + offset1, r4);
    float ball2 = sdSphere (p + offset2, r2);
	float metaBalls = opCombine (ball1, ball2, r1);

    ball1 = sdSphere (p + offset3, r1);
    ball2 = sdSphere (p + offset4, r3);
	metaBalls = opCombine (metaBalls, opCombine (ball1, ball2, .2), r2);

    ball1 = sdSphere (p + offset5, r3);
    ball2 = sdSphere (p + offset6, r2);
	metaBalls = opCombine (metaBalls, opCombine (ball1, ball2, .2), r3);

    ball1 = sdSphere (p + offset7, r3);
    ball2 = sdSphere (p + offset8, r4);
	metaBalls = opCombine (metaBalls, opCombine (ball1, ball2, .2), r4);

    return metaBalls;
}

float map (in vec3 p, out int id) {
    float metaBallsRibbons1 = metaballs (p, 1.005);
    float metaBallsRibbons2 = metaballs (p, 0.995);
    float metaBallsCore = metaballs (p, .4);
    float metaBallsRibbons = max (-metaBallsRibbons2, metaBallsRibbons1);
    vec3 boxCenter = p + iTime * vec3 (.0, .75, .0);
    opRepeat1 (boxCenter.y, .15);
    float box = udBox (boxCenter, vec3(12., mix (.015, .05, .5+.5*cos(125.*boxCenter.y)), 12.));
    metaBallsRibbons = max (metaBallsRibbons, box);
    float metaBalls = min (metaBallsCore, metaBallsRibbons);

    float room = sdBox (p, vec3 (150., 5., 150.), .0);
    float d = min (metaBalls, -room);

    id = 0;
    if (d == metaBallsCore) id = 1;
    if (d == metaBallsRibbons) id = 2;
    if (d == -room) id = 3;

    return d;
}

float march (in vec3 ro, in vec3 rd, inout int id) {
    float t = .0;
    float d = .0;
    int matId = 0;
    for (int i = 0; i < MAX_ITER; ++i) {
        vec3 p = ro + d * rd;
        t = map (p, matId);
        id = matId;
        if (abs (t) < EPSILON*(1. + .125*t)) break;
        d += t*STEP_BIAS;
    }

    return d;
}

vec3 normal (in vec3 p, in float epsilon) {
    int foo;
	float d = map (p, foo);
    vec2 e = vec2 (epsilon, .0);
    return normalize (vec3 (map (p + e.xyy, foo),
                            map (p + e.yxy, foo),
                            map (p + e.yyx, foo)) - d);
}

float shadow (in vec3 p, in vec3 lPos) {
    float lDist = distance (p, lPos);
    vec3 lDir = normalize (lPos - p);
    int foo;
    float dist = march (p, lDir, foo);
    return dist < lDist ? .1 : 1.;
}

float ao (in vec3 p, in vec3 n, float stepsize, int iterations, float intensity) {
    float ao = .0;
    float dist = .0;
    int foo;
    for (int a = 1; a <= iterations; ++a) {
        dist = float (a)*stepsize;
        ao += max (.0, (dist - map (p + n*dist, foo))/dist);
    }   
    return 1. - ao*intensity;
}

vec3 shade (in vec3 ro, in vec3 rd, in float d, in int id) {
    vec3 p = ro + d * rd;
    vec3 nor = normal (p, d*EPSILON);

    // "material" hard-coded for the moment
    float mask;
    vec3 albedo;
    float metallic;
    float roughness;
    float ao = ao (p, nor, .05, 8, .1);

    if (id == 1) {
        albedo = vec3 (.9, .3, .1);
	    metallic = .7;
    	roughness = .3;
    } else if (id == 2) {
        albedo = vec3 (.1, .3, .8);
	    metallic = .2;
    	roughness = .8;
    } else {
        albedo = vec3 (.0);
	    metallic = .0;
    	roughness = .0;
    }

    // lights hard-coded as well atm
    vec3 lightColors[2];
    lightColors[0] = vec3 (.7, .8, .9)*6.;
    lightColors[1] = vec3 (.9, .8, .7)*9.;

    vec3 lightPositions[2];
    lightPositions[0] = vec3 (-1.5, 1.0, -3.);
    lightPositions[1] = vec3 (2., -.5, 3.);

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
        float ldist = length(lightPositions[i] - p);
        float attenuation = 7. / (ldist*ldist);
        vec3 radiance = lightColors[i] * attenuation;
        
        // cook-torrance brdf
        float aDirect = pow (roughness + 1., 2.);
        float aIBL =  roughness * roughness;
        float NDF = distriGGX(N, H, roughness);        
        float G   = geomSmith(N, V, L, roughness);      
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
        if (SHADOW) {
	    	Lo *= shadow (p+.01*N, L);
        }
    }

    vec3 diffuse = albedo;
    vec3 ambient = (kD * diffuse) * ao;

    return ambient + Lo;
}

vec3 camera (in vec2 uv, in vec3 ro, in vec3 aim, in float zoom) {
    vec3 camForward = normalize (vec3 (aim - ro));
    vec3 worldUp = vec3 (.0, 1., .0);
    worldUp.xy *= r2d (35.*sin (.5+iTime));
    vec3 camRight = normalize (cross (worldUp, camForward));
    vec3 camUp = normalize (cross (camForward, camRight));
    vec3 camCenter = ro + camForward * zoom;
    
    return normalize (camCenter + uv.x * camRight + uv.y * camUp - ro);
}

vec3 background (in vec2 uv) {
    vec2 p = uv*iResolution.xy - ceil (.5*iResolution.xy);
    p *= r2d (34.*iTime);
    vec3 col = vec3 (.2, .5, 1.);

    col += .2*step (1. - 1./50., fract (p.x / 50.));
    col += .1*step (1. - 1./10., fract (p.x / 10.));
    col += .2*step (1. - 1./50., fract (p.y / 50.));
    col += .1*step (1. - 1./10., fract (p.y / 10.));

    return col;
}

void main () {
	vec2 uv = fragCoord.xy;
    vec2 uvRaw = uv;

    // set up "camera", view origin (ro) and view direction (rd)
    float angle = radians (300. + 55. * iTime);
    float dist = 2.75 + cos (1.5*(iTime + 2.));
    vec3 ro = vec3 (dist * cos (angle), cos (iTime), dist * sin (angle));
    vec3 aim = vec3 (.0);
    float zoom = 2.;
	float d = .0;
	vec3 col = vec3 (.0);    
    int id = 0;

	// walk over AA-grid
	for (int x = 0; x < AA_SIZE; ++x) {
		for (int y = 0; y < AA_SIZE; ++y) {

            // anti-alias offset
			vec2 pixelOffset = vec2 (float (x), float (y))/float (AA_SIZE);

            // normalize and aspect-correct UVs
			uv = (fragCoord.xy + pixelOffset/iResolution.xy);
    		uv = uv*2. - 1.;
            uv.x *= iResolution.x/iResolution.y;
            uv *= 1. + .5*length (uv);

    		// create viewray
    		vec3 rd = camera (uv, ro, aim, zoom);


            d = march (ro, rd, id);
            vec3 n = normal (ro + d * rd, d*EPSILON);
            vec3 ctmp = shade (ro, rd, d, id);
            col += ctmp;
        }
    }
	col /= float (AA_SIZE*AA_SIZE);
    if (id == 3) col = background (uvRaw);

	col = col / (1. + col);
    col *= 1. - .5*length (uvRaw*2. - 1.);
	col = pow (col , vec3 (1./2.2));

	fragColor = vec4 (col, 1.);
}
