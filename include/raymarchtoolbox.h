#ifndef _RAYMARCHING_TOOLBOX_H
#define _RAYMARCHING_TOOLBOX_H

#include "vec3.h"

using Color = std::array<float, 3>;
using UV = std::array<float, 2>;
using Seconds = float;
using Resolution = std::array<float, 2>;

float clamp (const float value, const float lower, const float upper);
float saturate (const float value);
float smoothstep (const float value, const float lower, const float upper);
float point3d (const vec3& ro, const vec3& rd, const vec3& p, float r);
float sdBall (const vec3& center, float radius);
float sdPlane (const vec3& point, float height);
float scene (const vec3& point);
float trace (const vec3& ro, const vec3& rd);
vec3 normal (const vec3& surfacePoint);
vec3 shade (const vec3& ro, const vec3& rd, float t);
Color computeColor (const UV& uv,
                    const Seconds& seconds,
                    const Resolution& res);

#endif
