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

// the HW_PERFORMANCE-check should take care of some optimization, if that does
// not help you can still force GROUND_WAVES to 0 and/or AA_SIZE to 1 manually
#if HW_PERFORMANCE==0
#define GROUND_WAVES 0
const int AA_SIZE = 1;
#else
#define GROUND_WAVES 1
const int AA_SIZE = 2;
#endif

const float PI = 3.14159265359;
const int MAX_ITER = 96;
const float STEP_BIAS = .5;
const float EPSILON = .0001;
const float MAX_DIST = 12.;

float smin (float a, float b, float k)
{
    float h = clamp (.5 + .5*(b - a)/k, .0, 1.);
    return mix (b, a, h) - h*k*(1. - h); 
}

mat2 r2d (float deg)
{
    float rad = radians (deg);
    float c = cos (rad);
    float s = sin (rad);
    return mat2 (c, s, -s, c);
}

// using a slightly adapted implementation of iq's simplex noise from
// https://www.shadertoy.com/view/Msf3WH with hash(), noise() and fbm()
vec2 hash (in vec2 p)
{
    p = vec2 (dot (p, vec2 (127.1, 311.7)),
              dot (p, vec2 (269.5, 183.3)));

    return -1. + 2.*fract (sin (p)*43758.5453123);
}

float noise (in vec2 p)
{
    const float K1 = .366025404;
    const float K2 = .211324865;

    vec2 i = floor (p + (p.x + p.y)*K1);
    
    vec2 a = p - i + (i.x + i.y)*K2;
    vec2 o = step (a.yx, a.xy);    
    vec2 b = a - o + K2; 
    vec2 c = a - 1. + 2.*K2;

    vec3 h = max (.5 - vec3 (dot (a, a), dot (b, b), dot (c, c) ), .0);

    vec3 n = h*h*h*h*vec3 (dot (a, hash (i + .0)),
                           dot (b, hash (i + o)),
                           dot (c, hash (i + 1.)));

    return dot (n, vec3 (70.));
}

float fbm (in vec2 p, in int iters)
{
    mat2 rot = r2d (35.);
    float d = .0;
    float f = 1.;
    float fsum = .0;

    for (int i = 0; i < iters; ++i) {
        d += f*noise (p);
        fsum += f;
        f *= .5;
    }
    d /= fsum;

    return d;
}

float plane (vec3 p, float h)
{
    return p.y + h;
}

float ball (vec3 p, float r)
{
    return length (p) - r;
}

float cube (vec3 p, float size)
{
    return length (max (abs (p) - size, .0));
}

float gyroid (vec3 p, float scale, float thickness, float bias, vec2 modulation, vec2 offset)
{
    float modulate = (modulation.x > modulation.y) ? modulation.x : modulation.y;
    p *= scale;
    float d = dot (sin(p*modulation.x) + offset.x,
                   cos(p.yzx*modulation.x) + offset.y) - bias;

    return abs (d)/scale/modulate - thickness;
}

float sMinGyroid (vec3 p,
                  float r,
                  float scale,
                  float thickness,
                  float bias,
                  vec2 modulation,
                  vec2 offset,
                  float blend,
                  float distScale)
{
    float ball = ball (p, r);
    float gyroid = gyroid (p, scale, thickness, bias, modulation, offset)*distScale;
    return  smin (ball, gyroid, blend);
}

float scene (vec3 p, out int id)
{
    // doing the wavy ground with fbm() is a bit on the costly side
    vec3 groundP = p;
    groundP.x += iTime;
    float pk = 1.5;
    #if GROUND_WAVES 
    if( p.y<0.5 )
    pk += .25*fbm(groundP.xz, 2);
    #endif
    float ground = plane (p, pk);

    p.xz *= r2d (5.*iTime);
    p.yx *= r2d (7.*iTime);
    vec2 modulation = vec2 (1.2, .8);
    float thickness1 = .025 + .025*(cos (10.*iTime)*.5 + .5);
    float thickness2 = .05 + .05*(cos (13.*iTime)*.5 + .5);
    float thickness3 = .1 + .1*(cos (16.*iTime)*.5 + .5);
    float bias1 = 1.3;
    float bias2 = .3;
    float bias3 = .025;
    float scale = 7.;
    vec2 offset = vec2 (.1, .4);
    float sMinBlend = -.05;
    float distScale = .55;

    float r1 = 1.35 - .6*(cos (.5*iTime)*.5+.5);    // 1.35
    float r2 = 1.1 - .4*(cos (iTime + 1.)*.5+.5);   // 1.1
    float r3 = .7 + .2*(cos (2.*iTime + 2.)*.5+.5); // .7 + .2

    float g1 = sMinGyroid (p, r1, scale, thickness1, bias1, modulation, offset, sMinBlend, distScale);
    float g2 = sMinGyroid (p, r2, scale, thickness2, bias2, modulation, offset, sMinBlend, distScale);
    float g3 = sMinGyroid (p, r3, scale, thickness3, bias3, modulation, offset, sMinBlend, distScale);

    float d = ground;
    d = min (d, g1);
    d = min (d, g2);
    d = min (d, g3);

    if (d == ground) id = 0;
    if (d == g1) id = 1;
    if (d == g2) id = 2;
    if (d == g3) id = 3;

    return d;
}

float raymarch (vec3 ro, vec3 rd, out int iter, out int id)
{
    float d = .0;
    float t = .0;
    int i = 0;
    vec3 p = vec3 (.0);
    int ignoreId = 0;

    for (; i < MAX_ITER; ++i) {
        p = ro + d*rd;
        t = scene (p, id);
        if (abs (t) < EPSILON*(1. - .125*t) || d > MAX_DIST) {
            iter = i;
            break;
        }
        d += t*STEP_BIAS;
    }

    return d;
}

vec3 normal (vec3 p)
{
    int ignoreId = 0;
    vec2 e = vec2 (EPSILON, .0);
    float d = scene (p, ignoreId);

    vec3 n = normalize (vec3 (scene (p + e.xyy, ignoreId),
                              scene (p + e.yxy, ignoreId),
                              scene (p + e.yyx, ignoreId)) - d);
    return n;
}

float shadow (vec3 p, vec3 n, vec3 lPos, vec3 lDir)
{
    int ignoreIter = 0;
    int ignoreId = 0;
    float distToWorld = raymarch (p + .01*n, lDir, ignoreIter, ignoreId);
    float distToLight = distance (p, lPos);

    return distToWorld < distToLight ? .3 : 1.;
}

float ao (vec3 p, vec3 n, float stepSize, int iterations, float intensity)
{
  float ao = .0; 
  float dist = .0; 
  int ignoreId = 0;

  for (int a = 1; a <= iterations; ++a) {
    dist = float (a)*stepSize;
    ao += max (.0, (dist - scene (p + n*dist, ignoreId))/dist);
  }

  return 1. - ao*intensity;
}

vec3 shade (vec3 ro, vec3 rd, float d, vec3 n, int id)
{
    vec3 p = ro + d*rd;

    vec3 lPos1 = vec3 (3.*cos(iTime), 3., 3.*sin(.4*iTime));
    vec3 lDir1 = normalize (lPos1 - p);
    float lDist1 = distance (lPos1, p);
    float attn1 = 30. / (lDist1*lDist1);
    vec3 lColor1 = vec3 (1., .9, .3);

    vec3 lPos2 = vec3 (-2.*cos(.3*iTime), 3., 4.*sin(iTime));
    vec3 lDir2 = normalize (lPos2 - p);
    float lDist2 = distance (lPos2, p);
    float attn2 = 40. / (lDist2*lDist2);
    vec3 lColor2 = vec3 (.2, .4, 1.);

    vec3 amb = vec3 (.1); 
    float diff1 = max (dot (n, lDir1), .0);
    float diff2 = max (dot (n, lDir2), .0);
    vec3 h1 = normalize (lDir1 - rd);
    vec3 h2 = normalize (lDir2 - rd);
    float spec1 = pow (max (dot (h1, n), .0), 40.);
    float spec2 = pow (max (dot (h2, n), .0), 40.);

    float s1 = shadow (p, n, lPos1, lDir1);
    float s2 = shadow (p, n, lPos2, lDir2);

    float ao = ao (p, n, .05, 12, .1);

    bool isFloor = id == 0;
    float phase = cos (20.*(p.x + iTime));
    float mask = smoothstep (.005*d, .0025*d, .5 + .5*phase);
    vec3 floorMaterial = 1.5*mix (vec3(.8), vec3 (.2), mask);

    vec3 gyroidMaterial = vec3(.0);
    if (id == 1) gyroidMaterial = 1.5*vec3(.8, .2, .1);
    if (id == 2) gyroidMaterial = 2.*vec3(.8, .7, .2);
    if (id == 3) gyroidMaterial = 3.5*vec3(.8, .5, .3);

    vec3 diffMaterial = isFloor ? floorMaterial : gyroidMaterial;

    return amb + ao*(attn1*s1*(diff1*diffMaterial*lColor1 + spec1) +
                     attn2*s2*(diff2*diffMaterial*lColor2 + spec2));
}

vec3 camera (vec2 uv, vec3 ro, vec3 aim, float zoom)
{
    vec3 camForward = normalize (aim - ro);
    vec3 worldUp = vec3 (.0, 1., .0);
    vec3 camRight = normalize (cross (camForward, worldUp));
    vec3 camUp = normalize (cross (camRight, camForward));
    vec3 camCenter = normalize (ro + camForward*zoom);

    return normalize (camCenter + uv.x*camRight + uv.y*camUp - ro);
}

void main()
{
    // normalize and aspect-correct UVs
	//vec2 uv = fragCoord.xy/iResolution.xy;
    vec2 uv = fragCoord;
    uv *= 2. - 1.;
    uv.x *= iResolution.x/iResolution.y;

    // allow some yaw-orbit with the mouse
    vec2 yaw = .75*vec2 (PI*cos (4.*iMouse.x/iResolution.x),
                         PI*sin (4.*iMouse.x/iResolution.x));
    float pitch = 1.;

    // create origin/camera/view-ray
    vec3 ro = vec3 (yaw.x, pitch, yaw.y);
    vec3 aim = vec3 (.0, .0, .0);
    float zoom = 1.75;
    vec3 rd = camera (uv, ro, aim, zoom);

    // raymarch, shading & floor-glow
    int iter = 0;
    float d = .0;
    vec3 p = vec3 (.0);
    vec3 n = vec3 (.0);
    vec3 color = vec3 (.0);
    float fog = .0;

    for (int x = 0; x < AA_SIZE; ++x) {
        for (int y = 0; y < AA_SIZE; ++y) {

			// anti-alias offset
			vec2 pixelOffset = vec2 (float (x), float (y))/float (AA_SIZE);

			// normalize and aspect-correct UVs
            uv = 2.*((fragCoord + pixelOffset)) - 1.;
            uv.x *= iResolution.x/iResolution.y;

    		// create viewray
    		rd = camera (uv, ro, aim, zoom);

			// primary/view ray
            int id = 0;
			d = raymarch (ro, rd, iter, id);
            p = ro + d*rd;
            n = normal (p);
			vec3 ctmp = shade (ro, rd, d, n, id);
            fog = 1. / (1. + d*d*.1);
            ctmp *= fog;
            ctmp = mix (ctmp, .5*vec3 (.15, .4, .9), pow (1. - 1./d, 6.));

			color += ctmp;
        }
    }
    color /= float (AA_SIZE*AA_SIZE);

    // make the final picture 'pretty'
    color = color / (1. + color);
    color *= 1. - .25*dot (uv, uv);
    color = .2*color + .8*sqrt (color);

    fragColor = vec4(color, 1.);
}
