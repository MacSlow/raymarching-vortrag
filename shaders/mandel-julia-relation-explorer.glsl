#version 420
uniform vec3 iResolution;
uniform float iTime;
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

const float PI = 3.14159265358979323846;
const vec4 red     = vec4 (1.0, 0.0, 0.0, 1.0);
const vec4 green   = vec4 (0.0, 1.0, 0.0, 1.0);
const vec4 blue    = vec4 (0.0, 0.0, 1.0, 1.0);
const vec4 white   = vec4 (1.0, 1.0, 1.0, 1.0);
const vec4 orange  = vec4 (1.0, 0.4, 0.125, 1.0);
const vec4 black   = vec4 (0.0, 0.0, 0.0, 1.0);
const vec4 cyan    = vec4 (0.0, 1.0, 1.0, 1.0);
const vec4 magenta = vec4 (1.0, 0.0, 1.0, 1.0);
const vec4 yellow  = vec4 (1.0, 1.0, 0.0, 1.0);
const float MAX_ITER = 256.;
const float LENGTH_LIMIT = 5.;
const bool SHOW_ORBIT = true;
const float THICKNESS = 0.0075;

vec4 line (vec2 p, vec2 a, vec2 b, vec4 c, float thickness)
{
    vec2 pa = -p - a;
    vec2 ba = b - a;
    float h = clamp (dot (pa, ba) / dot (ba, ba), 0.0, 1.0);
    float d = length (pa - ba * h);
    
    return c * clamp (((1.0 - d) - (1.0 - thickness)) * 100.0, 0.0, 1.0);
}

vec4 gradient (float v) {
    float steps = 7.;
    float step = 1. / steps;
    vec4 col = black;

    if (v >= .0 && v < step) {
        col = mix (yellow, orange, v * steps);
    } else if (v >= step && v < 2.0 * step) {
        col = mix (orange, red, (v - step) * steps);
    } else if (v >= 2.0 * step && v < 3.0 * step) {
        col = mix (red, magenta, (v - 2.0 * step) * steps);
    } else if (v >= 3.0 * step && v < 4.0 * step) {
        col = mix (magenta, cyan, (v - 3.0 * step) * steps);
    } else if (v >= 4.0 * step && v < 5.0 * step) {
        col = mix (cyan, blue, (v - 4.0 * step) * steps);
    } else if (v >= 5.0 * step && v < 6.0 * step) {
        col = mix (blue, green, (v - 5.0 * step) * steps);
    }

    return col;
}

vec4 calcMandel (vec2 c)
{
    vec2 z = vec2 (.0);
    float iter = .0;

    for (float i = 0.; i < MAX_ITER; i += 1.) {
        z = mat2 (z, -z.y, z.x) * z + c;
        if (length (z) > LENGTH_LIMIT && iter == .0) {
            iter = i;
        }
    }

    return length (z) <= LENGTH_LIMIT ? vec4 (0) : gradient (iter / MAX_ITER);
}

vec4 mandel (vec2 p, vec2 size)
{
    // ordered 2x2-grid super-sampling
    return (  calcMandel (p + size * vec2 (-.5, -.5))
            + calcMandel (p + size * vec2 ( .5, -.5))
            + calcMandel (p + size * vec2 ( .5,  .5))
            + calcMandel (p + size * vec2 (-.5,  .5))) / 4.;
}

vec4 calcJulia (vec2 p, vec2 c)
{
    vec2 z = p;
    float iter = .0;

    for (float i = 0.; i < MAX_ITER; i+= 1.) {
        z = mat2 (z, -z.y, z.x) * z + c;
        if (length (z) > LENGTH_LIMIT && iter == .0) {
            iter = i;
        }
    }
    return length (z) <= LENGTH_LIMIT ? vec4 (0) : gradient (iter / MAX_ITER);
}

vec4 julia (vec2 p, vec2 size, vec2 c)
{
    // ordered 2x2-grid super-sampling
    return (  calcJulia (p + size * vec2 (-.5, -.5), c)
            + calcJulia (p + size * vec2 ( .5, -.5), c)
            + calcJulia (p + size * vec2 ( .5,  .5), c)
            + calcJulia (p + size * vec2 (-.5,  .5), c)) / 4.;
}

vec4 morbit (vec2 p, vec2 s, float thickness)
{
    vec4 result = vec4 (.0);
    vec2 z = vec2 (.0);
    vec2 zNew = vec2 (.0);
    vec2 c = s;

    for (float i = .0; i < MAX_ITER; i+= 1.) {
        zNew = mat2 (z, -z.y, z.x) * z + c;
	    result += line (p, vec2 (-z.x, z.y), vec2 (-zNew.x, zNew.y), vec4 (1.), thickness);
        z = zNew;
    }
    return result;
}

void main()
{
	vec2 p = fragCoord;
    vec2 res = iResolution.xy;
    vec2 uv = p;
    vec2 nuv = -1. + 2. * uv;
    float aspect = iResolution.x / iResolution.y;
    nuv.y /= aspect;

    float s = 3.;
    vec2 moffset = s * vec2 (1./4., .0);
    vec2 joffset = s * vec2 (1./6., .0);
    vec2 pmandel = vec2 (nuv) - moffset;
    vec2 pjulia = vec2 (nuv) + joffset;

    vec2 m = vec2 (iMouse.x / iResolution.x, iMouse.y / iResolution.y);
    vec2 nc = -1. + 2. * m;
    vec2 mc = nc - moffset;
    mc *= s;
    mc.y /= aspect;
    //mc.y *= -1.;

    if (nuv.x > .0) {
    	fragColor = mandel (s * pmandel , s/res);
    } else {
    	fragColor = julia (s * pjulia , s/res, mc);
    }

    if (SHOW_ORBIT) {
        fragColor += morbit (s * pmandel, mc, THICKNESS);
    }
}

