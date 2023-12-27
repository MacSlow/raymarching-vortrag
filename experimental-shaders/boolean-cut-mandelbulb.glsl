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

const int MAX_ITER = 96;
const float STEP_SIZE = .75;
const float EPSILON = .001;
const float PI = 3.1415926;

struct Ray {
    vec3 ro;
    vec3 rd;
};

struct Result {
    vec3 point;
    vec3 normal;
    float dist;
    int iter;
    int id;
};

const Result nullResult = Result (vec3 (.0), vec3 (.0), .0, 0, -1);

mat2 r2d (in float a)
{
    float c = cos (radians (a));
    float s = sin (radians (a));
    return mat2 (c, s, -s, c);
}

float mandelbulb (vec3 pos, in float n, in float bail, in int iter)
{
    vec3 z = pos;
    float dr = 1.;
    float r = .0;
    for (int i = 0; i < iter; i++) {
        r = length (z);
        if (r > bail) break;

        // from cartesian to polar
        float theta = acos (z.z / r);
        float phi = atan (z.y, z.x);
        dr = pow(r, n - 1.) * n * dr + 1.;

        // scale and rotate the point
        float zr = pow (r, n);
        theta = theta * n;
        phi = phi * n;

        // back to cartesian
        z = zr * vec3(sin(theta)*cos(phi), sin(theta)*sin(phi), cos(theta));
        z += pos;
    }

    return .5 * log (r) * r / dr; // I just don't get this distance estimator here
}

float sdBox (in vec3 p, in vec3 size)
{
    vec3 d = abs (p) - size;
    return min (max (d.x, max (d.y, d.z)), .0) + length (max (d, .0));
}

float map (in vec3 p)
{
    float d = mandelbulb (p, 8. + 8.*(.5 + .5*cos(iTime)), 4., 8);
    p.xz *= r2d (45.*iTime);
    p.yz *= r2d (75.*iTime);
    float box = sdBox (p + vec3 (-.75*(.5 + .5*cos(iTime)), -.5, .0), vec3 (1., .5, .5 + 1.*(.5 + .5*cos (iTime))));

    return max (d, -box);
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

    return ggx1 * ggx2;
}

vec3 fresnelSchlick (in float cosTheta, in vec3 F0, float roughness)
{
    return F0 + (max (F0, vec3(1. - roughness)) - F0) * pow (1. - cosTheta, 5.);
}

vec3 normal (in vec3 p, in float epsilon)
{
    float d = map (p);
    vec2 e = vec2 (epsilon, .0);
    vec3 n = vec3 (map (p + e.xyy) - d,
                   map (p + e.yxy) - d,
                   map (p + e.yyx) - d);

    return normalize (n);
}

vec3 shadePBR (in vec3 ro, in vec3 rd, in float d)
{
    vec3 p = ro + d * rd;
    vec3 nor = normal (p, d*EPSILON);

    // "material" hard-coded for the moment
    vec3 albedo = vec3 (.25);
    float metallic = .35;
    float roughness = .25;

    // lights hard-coded as well atm
    vec3 lightColors[2];
    lightColors[0] = vec3 (.7, .4, .2)*7.;
    lightColors[1] = vec3 (.2, .7, .4)*9.;

    vec3 lightPositions[2];
    float t = iTime;
    float c = cos (t);
    float s = sin (t);
    lightPositions[0] = vec3 (1.14, 1.5, -1.75);
    lightPositions[1] = vec3 (-.125, 1.5, 1.75);

    vec3 N = normalize (nor);
    vec3 V = normalize (ro - p);

    vec3 F0 = vec3 (.04);
    F0 = mix (F0, albedo, metallic);
    vec3 kD = vec3 (.0);

    // reflectance equation
    vec3 Lo = vec3 (.0);
    for (int i = 0; i < 2; ++i)
    {
        // calculate per-light radiance
        vec3 L = normalize (lightPositions[i] - p);
        vec3 H = normalize (V + L);
        float dist = distance (p, lightPositions[i]);
        float attenuation = 4./(dist*dist);
        vec3 radiance = lightColors[i]*attenuation;

        // cook-torrance brdf
        float aDirect = pow (roughness + 1., 2.);
        float aIBL =  roughness * roughness;
        float NDF = distriGGX (N, H, roughness);
        float G = geomSmith (N, V, L, roughness);
        vec3 F = fresnelSchlick (max (dot (H, V), .0), F0, roughness);

        vec3 kS = F;
        kD = vec3 (1.) - kS;
        kD *= 1. - metallic;

        vec3 nominator = NDF * G * F;
        float denominator = 4. * max (dot (N, V), .0) * max (dot (N, L), .0);
        vec3 specular = nominator / max (denominator, .001);

        // add to outgoing radiance Lo
        float NdotL = max (dot (N, L), .0);
        Lo += (kD*albedo/PI + specular)*radiance*NdotL;
    }

    vec3 ambient = kD * albedo;

    return ambient + Lo;
}

Result raymarch (in Ray ray)
{
    Result res = nullResult;

    for (int i = 0; i < MAX_ITER; ++i) {
        res.iter = i;
        float tmp = map (ray.ro + res.dist * ray.rd);
        if (tmp < EPSILON) break;
        res.dist += tmp * STEP_SIZE;
    }

    res.point = ray.ro + res.dist * ray.rd;
    res.normal = normal (res.point, EPSILON*res.dist);

    return res;
}

float shadow (in Ray ray, in vec3 lPos)
{
    float distToLight = distance (lPos, ray.ro);
    float dist = .0;

    for (int i = 0; i < MAX_ITER; ++i) {
        float tmp = map (ray.ro + dist * ray.rd);
        if (tmp < EPSILON) {
            if (dist < distToLight)
                return .125;
            else
                return 1.;
        }
        dist += tmp * STEP_SIZE;
    }

    return 1.;
}

void main ()
{
    vec2 uv = fragCoord.xy;
    uv = uv * 2. - 1.;
    uv.x *= iResolution.x / iResolution.y;

    float r = 1.5 + .5 * (cos (iTime)*.5+.5);
    float theta = (iMouse.x / iResolution.x) * PI;
    float phi = -(iMouse.y / iResolution.y * 2. - 1.) * PI;
    float x = r * sin (theta) * cos (phi);
    float y = r * sin (theta) * sin (phi);
    float z = r * cos (theta);
    vec3 ro = vec3 (x, y, z);
    float zoom = 2.;
    vec3 lookAt = vec3 (.0);
    vec3 forward = normalize (lookAt - ro);
    vec3 worldUp = vec3 (.0, 1., .0);
    vec3 right = normalize (cross (worldUp, forward));
    vec3 up = normalize (cross (forward, right));
    vec3 camCenter = ro + zoom * forward;
    vec3 i = camCenter + uv.x * right + uv.y * up;
    vec3 rd = normalize (i - ro);

    Ray ray = Ray (ro, rd);
    Result res = raymarch (ray);
    vec3 col = shadePBR (ro, rd, res.dist);
    float fog = float (res.iter) / float (MAX_ITER);
    col *= 1. - (fog * fog);

    col = col / (.85 + col);
    col *= vec3 (.95, .9, .85);
    col = .3*col + .7*sqrt (col);

    fragColor = vec4 (col, 1.);
}
