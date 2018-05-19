#version 420
uniform vec3 iResolution;
uniform float iTime;
uniform vec4 iMouse;
in vec2 fragCoord;
out vec4 fragColor;

const float EPSILON = .0001;

float hash (float f)
{
	return fract (sin (f) * 45734.5453);
}

float noise3d (vec3 p)
{
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

    float result = mix (mix (mix (a, b, v.x),
                             mix (c, d, v.x),
                             v.y),
                        mix (mix (e, f, v.x),
                             mix (g, h, v.x),
                             v.y),
                        v.z);

    return result;
}

mat3 rotY (in float angle)
{
    float rad = radians (angle);
    float c = cos (rad);
    float s = sin (rad);

    mat3 mat = mat3 (vec3 ( c, .0, -s),
                     vec3 (.0, 1,  .0),
                     vec3 ( s, .0,  c));

    return mat;
}

mat3 rotZ (in float angle)
{
    float rad = radians (angle);
    float c = cos (rad);
    float s = sin (rad);

    mat3 mat = mat3 (vec3 (  c,  -s, 0.0),
                     vec3 (  s,   c, 0.0),
                     vec3 (0.0, 0.0, 1.0));

    return mat;
}

float fbm (vec3 p)
{
	mat3 m1 = mat3 (rotZ (23.4));
	mat3 m2 = mat3 (rotZ (45.5));
	mat3 m3 = mat3 (rotZ (77.8));

    float result = .0;
    result = 0.5 * noise3d (p);
    p *= m1 * 2.02;
    result += 0.25 * noise3d (p);
    p *= m2 * 2.03;
    result += 0.125 * noise3d (p);
    p *= m3 * 2.04;
    result += 0.0625 * noise3d (p);
    result /= 0.9375;

    return result;
}

float sphere (vec3 p, float size)
{
	return length (p) - size;
}

float plane (vec3 p)
{
    return p.y + .25;
}

float opBend (inout vec3 p, float deg)
{
    float rad = radians (deg);
    float c = cos (rad * p.y);
    float s = sin (rad * p.y);
    mat2  m = mat2 (c, -s, s, c);
    p = vec3 (m * p.xy, p.z);
    return .0;
}

float displace (vec3 p)
{
    float result = 1.;
	float factor = 2.;
    result =  sin (factor * p.x) * cos (factor * p.y) * sin (factor * p.z);
    return result;
}

vec2 scene (vec3 p)
{
    float dt = .0;
    float dp = .0;
	vec3 w = vec3 (.0);
	vec2 d1 = vec2 (.0);
	mat3 m = rotY (20. * iTime) * rotZ (-20. * iTime);

    // floor
    vec2 d2 = vec2 (plane (p), 2.);

    // blue thingy
    w = -m * (p + vec3 (.0, -1.5, .0));
    opBend (w, 7. * cos (iTime));
	d1.y = 3.;
	float thickness = .075;
	float cubeSize = 6.;
	float surfaceSide = dot (sin (w), cos (w.yzx));
	d1.x = abs (surfaceSide) - thickness;
	vec3 a = abs (w);
	d1.x = max (d1.x, max (a.x, max (a.y, a.z)) - cubeSize);

	if (d2.x < d1.x) {
	    d1 = d2;
	}

	return d1;
}

vec3 normal (vec3 p, float epsilon)
{
	vec3 n;
    vec2 e = vec2 (epsilon, .0);
	float d = scene (p).x;
    n.x = scene (p + e.xyy).x - d;
    n.y = scene (p + e.yxy).x - d;
    n.z = scene (p + e.yyx).x - d;
    return normalize (n);
}

const int MAX_STEPS = 128;
const float SMALL_STEP_SIZE = .125;
const float LARGE_STEP_SIZE = .5;

vec2 raymarch (vec3 ro, vec3 rd)
{
    float pixelSize = 1. / iResolution.x;
    bool forceHit = true;
    float infinity = 10000000.0;
    float t_min = .0000001;
    float t_max = 1000.0;
    float t = t_min;
    vec2 candidate = vec2 (t_min, .0);
    vec2 candidate_error = vec2 (infinity, .0);
    float w = LARGE_STEP_SIZE;
    float lastd = .0;
    float stepSize = .0;
    float sign = scene (ro).x < .0 ? -1. : 1.;

    for (int i = 0; i < MAX_STEPS; i++)
	{
        float signedd = sign * scene (ro + rd * t).x;
        float d = abs (signedd);
        bool fail = w > 1. && (d + lastd) < stepSize;

        if (fail) {
            stepSize -= w * stepSize;
            w = SMALL_STEP_SIZE;
        } else {
            stepSize = signedd * w;
        }

		lastd = d;

        float error = d / t;
        if (!fail && error < candidate_error.x) {
            candidate_error.x = error;
            candidate.x = t;
        }

        if (!fail && error < pixelSize || t > t_max) {
        	break;
		}

        candidate_error.y = scene (ro + rd * t).y;
        candidate.y = candidate_error.y;

        t += stepSize;
	}

	if ((t > t_max || candidate_error.x > pixelSize) && !forceHit) {
	    return vec2 (infinity, .0);
	}

	return candidate;
}

float shadow (vec3 ro, vec3 rd)
{
    float result = 1.;
    float t = .1;
    for (int i = 0; i < 64; i++) {
        float h = scene (ro + t * rd).x;
        if (h < .00001) return .0;
        result = min (result, 4. * h/t);
        t += h*.5;
    }

    return result;
}

vec3 floorMaterial (vec3 pos, vec3 nor)
{
    vec3 col = vec3 (.6, .5, .3);
    float f = fbm (pos * vec3 (6., .0, .5));
    col = mix (col, vec3 (.3, .2, .1), f);
    f = smoothstep (.6, 1., fbm (48. * pos));
    col = mix (col, vec3 (.2, .2, .15), f);

    return col;
}

void main ()
{
	// normalizing and aspect-correcting UV-coords
    vec2 aspect = vec2 (iResolution.x/ iResolution.y, 1.);
    vec2 uv = fragCoord.xy;
    vec2 p = vec2 (-1. + 2. * uv) * aspect;

	// camera setup
    vec3 ro = 9. * vec3 (cos (.2 * iTime), 1.25, sin (.2 * iTime));
    vec3 ww = normalize (vec3 (.0, 1., .0) - ro);
    vec3 uu = normalize (cross (vec3 (.0, 1., .0), ww));
    vec3 vv = normalize (cross (ww, uu));
    vec3 rd = normalize (p.x * uu + p.y * vv + 1.5 * ww);

    vec2 t = raymarch (ro, rd);

    vec3 col = vec3 (.8);
    if (t.y > .5) {
        vec3 pos = ro + t.x * rd;
        vec3 nor = normal (pos, t.x*EPSILON);
        vec3 lig = normalize (vec3 (1., .8, .6));
        vec3 blig = normalize (vec3 (-lig.x, lig.y, -lig.z));
        vec3 ref = normalize (reflect (rd, nor));

        float con = 1.;
        float amb = .5 + .5 * nor.y;
        float diff = max (dot (nor, lig), .0);
        float bac = max (.2 + .8 * dot (nor, blig), .0);
        float sha = shadow (pos, lig);
        float spe = pow (clamp (dot (ref, lig), .0, 1.), 8.);
        float rim = pow (1. + dot (nor, rd), 2.5);

        col  = con  * vec3 (.1, .15, .2);
        col += amb  * vec3 (.1, .15, .2);
        col += diff * vec3 (1., .97, .85) * sha;
        col += spe * vec3 (.9, .9, .9);
        col += bac;

        if (t.y == 2.) {
            col *= floorMaterial (pos, nor);
        } else if (t.y == 3.) {
            col *= vec3 (.9, .4, .0);
        }

        col += .6 * rim * amb;
        col += .6 * spe * sha * amb;
    }

    // gamma-correction, tint, vignette
	col = col / (.85 + col);
    col *= vec3 (.95, .9, .85);
    col = .2 * col + .8 * sqrt (col);
    col *= .2 + .8 * pow (16.*uv.x*uv.y * (1. - uv.x) * (1. - uv.y), .2);

    fragColor = vec4(col, 1.);
}
