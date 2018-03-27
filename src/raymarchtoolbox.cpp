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

float clamp (const float v, const float edge0, const float edge1) {
    return (v < edge0) ? edge0 : (v > edge1) ? edge1: v;
}

float saturate (const float v) {
    return clamp (v, .0f, 1.f);
}

float smoothstep (const float v, const float edge0, const float edge1) {
    float t = clamp((v - edge0) / (edge1 - edge0), .0f, 1.f);
    return t * t * (3.f - 2.f * t);
}

float point3d (const vec3& ro, const vec3& rd, const vec3& p, float r)
{
    float d = length (cross (p - ro, rd)) / length (rd);
    return smoothstep (d, r - EPSILON, r);
}

float sdBall (const vec3& p, float r)
{
	return length (p) - r;
}

float sdPlane (const vec3& p, float h)
{
    return length (vec3 (.0f, p.y() + h, .0f));
}

float scene (const vec3& p)
{
	float res = .0f;

    float floor = sdPlane (p, .6f);
    float ball = sdBall (vec3 (p.x(), p.y(), p.z()) +
                         vec3 (-.3f, -.1f, -.1f),
                         .2f);

    res = min (ball, floor);

    return res;
}

float trace (const vec3& ro, const vec3& rd)
{
    float t = .0;

    for (int i = 0; i < MAX_ITER; i++)
    {
        vec3 p = ro + rd * t;
        float d = scene (p);
        if (d < EPSILON) break;
        t += d * STEP_SIZE;
    }

    return t;
}

vec3 normal (const vec3& p)
{
    float d = scene (p);
    vec3 ex = vec3 (.001f, .0f, .0f);
    vec3 ey = vec3 (.0f, .001f, .0f);
    vec3 ez = vec3 (.0f, .0f, .001f);

    return normalize (vec3 (scene  (p + ex) - d,
                            scene  (p + ey) - d,
                            scene  (p + ez) - d));
}

vec3 shade (const vec3& ro, const vec3& rd, float t)
{
    vec3 p = ro + rd * t;
    vec3 n = normal (p);
    vec3 ref = normalize (reflect (rd, n));
    float ambient = .2f;
    vec3 ambColor = vec3 (.15f, .1f, .1f);

    vec3 lightPos = p + vec3 (.4f, .35f, -.5f);
    vec3 lightDir = normalize (lightPos - p);
    vec3 diffColor = vec3 (.95f, .85f, .75f);
    float intensity = max (dot (n, lightDir), .0f);

    return (ambColor*ambient) + (diffColor*intensity);
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

Color computeColor (const UV& uv,
                    const Seconds& seconds,
                    const Resolution& res)
{
    Color color = {{uv[0], uv[1], .0f}};
    float aspect = res[0] / res[1];

    UV cuv {{aspect*(uv[0]*2.f - 1.f), -1.f*(uv[1]*2.f - 1.f)}};
    vec3 lookAt = vec3 (.0f);
    float zoom = 1.5f;
    auto [ro, rd] = createCamera (cuv, seconds, lookAt, zoom);

    float d = trace (ro, rd);
    vec3 n = shade (ro, rd, d);

    color[0] = n.x();
    color[1] = n.y();
    color[2] = n.z();

    return color;
}
