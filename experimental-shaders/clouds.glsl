// build/raymarcher-gl clouds.glsl 640 360 30
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

const int MAX_ITER = 128;
const float STEP_SIZE = .005;
const float FAR = .5;

struct Ray {
    vec3 ro;
    vec3 rd;
};

struct Result {
    float dist;
    float density;
};

const Result nullResult = Result (.0, .0);

mat2 r2d (in float a) { float r = radians (a); float c = cos (r); float s = sin (r); return mat2(vec2(c, s), vec2(-s, c));}
//float hash (float f) { return fract (sin (f)*4.5453); }
float hash (float f) { return fract (sin (f)*45843.349); }
float noise3d (vec3 p) {
    vec3 u = floor (p);
    vec3 v = fract (p);
    v = v * v * (3. - 2. * v);
    float n = u.x + u.y * 57. + u.z * 113.;
    float a = hash (n);
    float b = hash (n + 1.);
    float c = hash (n + 57.);
    float d = hash (n + 58.);
    float e = hash (n + 113.);
    float f = hash (n + 114.);
    float g = hash (n + 170.);
    float h = hash (n + 171.);
    return mix (mix (mix (a, b, v.x),
                     mix (c, d, v.x),
                     v.y),
                mix (mix (e, f, v.x),
                     mix (g, h, v.x),
                     v.y),
                v.z);
}

mat3 rotZ (float degree) {
	float r = radians (degree);
    float c = cos (r);
    float s = sin (r);
    mat3 m = mat3 (vec3 ( c,  s, .0),
                   vec3 (-s,  c, .0),
                   vec3 (.0, .0, .0));
    return m;
}

float fbm (vec3 p) {
	mat3 m1 = rotZ (1.1);
	mat3 m2 = rotZ (-2.2);
	mat3 m3 = rotZ (3.3);

    float result = .0;
    result = .5 * noise3d (p); p *= m1 * 2.02;
    result += .25 * noise3d (p); p *= m2 * 2.03;
    result += .125 * noise3d (p); p *= m3 * 2.04;
    result += .0625 * noise3d (p); p *= m2 * -2.05;
    result += .01325 * noise3d (p);
    result /= (.5 + .25 + .125 + .0625 + .01325);

    return result;
}

Result march (in Ray ray, in bool typeToggle) {
	float max_density = .0;
	float density = .0;
    Result res = nullResult;
    for (int i = 0; i < MAX_ITER; ++i) {
        vec3 p = ray.ro + res.dist*ray.rd;
        p.xy *= r2d (2.*iTime);
        p.yz *= r2d (-3.*iTime);
        p.zx *= r2d (4.*iTime);
        if (typeToggle) {
        	//res.density += .01*noise3d (12.*p);
        	density = .75*noise3d (17.*p);
        } else {
        	//res.density += .01*fbm (12.*p);
        	density = .75*fbm (17.*p);
        }
		max_density = max_density < density ? density : max_density;
        if (max_density > 1. || res.dist > FAR) break;
        res.dist += STEP_SIZE;// + .01*noise3d (20.*p);
    }

	res.density = max_density;
    return res;
}

Ray camera (in vec3 ro, in vec3 aim, in float zoom, in vec2 uv) {
	vec3 forward = normalize (aim - ro);
	vec3 worldUp = vec3 (.0, 1., .0);
	vec3 right = normalize (cross (forward, worldUp));
	vec3 up = normalize (cross (right, forward));
	vec3 center = normalize (ro + forward*zoom);

    return Ray (ro, normalize ((center + uv.x*right + uv.y*up) - ro));
}

void main () {
	vec2 uv = fragCoord.xy;
    uv = uv * 2. - 1.;
    uv.x *= iResolution.x / iResolution.y;

    float t = .00625*iTime;
    vec3 ro = vec3 (3.*cos (t), sin (t), 3.*sin (t));
    vec3 lookAt = vec3 (.0);
    float zoom = 2.75;
    Ray ray = camera (ro, lookAt, zoom, uv);
    Result res;
    if (uv.x < .0) {
    	res = march (ray, true);
    } else {
        res = march (ray, false);
    }
    vec3 col = mix (vec3(.0), vec3(1., .95, .8), res.density*res.density);

    col = col / (1. + col);
    col = .3*col + .7*sqrt (col);

	fragColor = vec4 (col, 1.);
}
