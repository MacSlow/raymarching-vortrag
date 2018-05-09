#ifndef _RAYMARCHING_TOOLBOX_H
#define _RAYMARCHING_TOOLBOX_H

#include "vec3.h"

using Color = std::array<float, 3>;
using UV = std::array<float, 2>;
using Seconds = float;
using Resolution = std::array<float, 2>;
using Mouse = std::array<float, 2>;

float clamp (float value, float lower, float upper);
float fract (float x);
float mix (float x, float y, float a);
float smin (float d1, float d2, float r);
float saturate (float value);
float smoothstep (float value, float lower, float upper);
float sdBall (const vec3& center, float radius);

Color computeColor (const UV& uv,
                    const Seconds& seconds,
                    const Resolution& res,
                    const Mouse& mouse);

#endif
