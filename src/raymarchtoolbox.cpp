#include <array>
#include <algorithm>
#include <tuple>
#include <cmath>

#include "raymarchtoolbox.h"

#define MAX_ITER 48
#define STEP_SIZE .95f
#define EPSILON .00125f

using vec2 = float[2];
using std::min;
using std::max;
using std::make_tuple;
using std::tuple;
using std::modf;

float clamp (float v, float edge0, float edge1) {
    return (v < edge0) ? edge0 : (v > edge1) ? edge1: v;
}

float fract (float x)
{
	float ignored = .0f;
	return modf (x, &ignored);
}

float mix (float x, float y, float a)
{
	return x*(1.f - a) + y*a;
}

float smin (float d1, float d2, float r)
{
    float h = clamp (.5f + .5f * (d2 - d1) / r, .0f, 1.f);
    return mix (d2, d1, h) - r * h * (1.f - h);
}

float saturate (float v) {
    return clamp (v, .0f, 1.f);
}

float smoothstep (float v, float edge0, float edge1) {
    float t = clamp((v - edge0) / (edge1 - edge0), .0f, 1.f);
    return t * t * (3.f - 2.f * t);
}

float sdBall (const vec3& p, float r)
{
	return length (p) - r;
}

tuple<vec3, vec3> createCamera (const UV& uv,
                                const Seconds seconds,
                                const vec3& lookAt,
                                float zoom)
{
    vec3 ro {3.f * cosf (seconds),
             2.f + .5f * cosf (seconds),
             3.f * sinf (seconds)};
    vec3 camForward = normalize (lookAt - ro);
    vec3 worldUp = vec3 (.0f, 1.f, .0f);
    vec3 camRight = cross (worldUp, camForward);
    vec3 camUp = cross (camForward, camRight);
    vec3 camCenter = ro + camForward * zoom;
    vec3 i = camCenter + camRight*uv[X] + camUp*uv[Y];
    vec3 rd = i - ro;

    return make_tuple (ro, rd);
}

Color computeColor (const UV& uv, const Seconds& seconds, const Resolution& res,
                    const Mouse& mouse)
{
    Color color = {{uv[0], uv[1], .0f}};
    float aspect = res[0] / res[1];

    UV cuv {{aspect*(uv[X]*2.f - 1.f)*4.f, -1.f*(uv[Y]*2.f - 1.f)*4.f}};
    vec3 m {aspect*(mouse[X]/res[X]*2.f - 1.f)*4.f, -1.f*(mouse[Y]/res[Y]*2.f - 1.f)*4.f, .0f};

    vec3 p = vec3 (cuv[X], cuv[Y], .0f); 
    float b1 = sdBall (p - vec3 (2.f*cos (seconds), 2.f*sin (seconds), .0f), 1.25f);
    float b2 = sdBall (p - m, 2.f);
    float d = smin (b1, b2, .75f);
    // float d = min (b1, b2);

    float f = fract (d);

    if (d > .0f) {
        float c = 1.f - smoothstep (.025f, .0125f, f);
		color[0] = c;
		color[1] = c;
		color[2] = c;
    } else {
		color[0] = .0f;
        color[1] = .5f;
        color[2] = .0f;
    }

    return color;
}
