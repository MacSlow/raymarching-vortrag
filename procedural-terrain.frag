#version 130
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

float hash (float f)
{
	return fract (sin (f) * 45785.5453);
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

mat3 rotZ (float degree)
{
	float r = radians (degree);
    float c = cos (r);
    float s = sin (r);
    mat3 m = mat3 (vec3 ( c,  s, .0),
                   vec3 (-s,  c, .0),
                   vec3 (.0, .0, .0));

    return m;
}

float fbm (vec3 p)
{
	mat3 m1 = rotZ (1.1);
	mat3 m2 = rotZ (-1.2);
	mat3 m3 = rotZ (1.0);

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

vec3 noise2d (in vec2 p)
{
    return vec3 (fbm (vec3 (p, .0)));
}

float mi;
float terrain (in vec3 p)
{
    float h = noise2d (.25 * p.xz).x * 1.75;
    float h2 = noise2d (.5* p.xz).x * .2;
    mi = .0;
    float d = p.y - (h*h+h2);
    float d2 = p.y;
    if (d2 < d) {d = d2; mi = 1.;}
    return d;
}

vec2 e = vec2 (.01, .0);

vec3 normal (in vec3 p)
{
    return normalize (vec3 (terrain (p + e.xyy),
                            terrain (p + e.yxy),
                            terrain (p + e.yyx)) - terrain (p));
}

float trace (in vec3 ro, in vec3 rd, in float t, in float tmax)
{
    float d = t;
    for (int i = 0; i < 64; ++i) {
        d = terrain (ro + t * rd);
        t += d;
        if (d < .01 * t || t >= tmax) break;
    }
    return t;
}

float shade (in vec3 p, in vec3 n)
{
    vec3 lPos = vec3 (cos(iTime)*2., 1., sin(iTime)*2. - iTime);
	vec3 lDir = normalize (lPos - p);
	return max (dot (n, lDir), .0);
}

vec3 ro, rd, p, n;

float mshi = 100., mkd=.5;
float dirlight (in vec3 ld)
{
	return mix (
        (mshi + 8.) * 25. * pow (max (.0, dot (n, normalize (ld - rd))), mshi),
        max (.0, dot (n, ld)) / 3.,
        mkd);
}


void main()
{
	vec2 uv = fragCoord.xy;
    uv = uv * 2. - 1.;
    uv.x *= iResolution.x / iResolution.y;

    ro = vec3 (.0, 2.75, 1. - iTime);
    rd = normalize (vec3 (uv, -1.));
    float t = .1;
    float tmax = 20.0;
    vec3 col = vec3(.0);
    vec3 kc = vec3 (1.);

    for (int i = 0; i < 2; ++i) {
	    float d = trace (ro, rd, t, tmax);
        if (d > tmax) { col += vec3 (.2, .4, .8); break; }
        p = ro + d * rd;
        n = normal (p);

        vec3 lPos = vec3 (1.);
		vec3 lDir = normalize (lPos - p);

        col += kc * dirlight (lPos);
    }

    fragColor = vec4 (sqrt (col), 1.);
}
