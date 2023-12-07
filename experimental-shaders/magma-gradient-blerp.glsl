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

const vec3 col0 = vec3 (111./255., 204./255., 221./255.);
const vec3 col1 = vec3 ( 51./255., 196./255., 243./255.);
const vec3 col2 = vec3 ( 67./255., 143./255., 205./255.);
const vec3 col3 = vec3 ( 63./255., 116./255., 186./255.);
const vec3 col4 = vec3 ( 54./255.,  79./255., 161./255.);
const vec3 col5 = vec3 ( 88./255.,  62./255., 153./255.);
const vec3 col6 = vec3 (112./255.,  36./255., 111./255.);
const vec3 col7 = vec3 (217./255.,  25./255.,  62./255.);
const vec3 col8 = vec3 (237./255.,  31./255.,  35./255.);
const vec3 col9 = vec3 (247./255., 112./255.,  31./255.);
const vec3 colA = vec3 (249./255., 164./255.,  21./255.);
const vec3 colB = vec3 (252./255., 204./255.,   8./255.);
const vec3 colC = vec3 (255./255., 228./255.,   4./255.);
const vec3 colD = vec3 (245./255., 235./255.,  16./255.);
const vec3 colE = vec3 (249./255., 244./255., 176./255.);
const vec3 colF = vec3 (253./255., 253./255., 253./255.);

vec3 gradient (float v)
{
    float steps = 16.;
    float step = 1./steps;
    vec3 col = vec3 (1., .0, 1.); // something nasty as default

    if (v >= .0 && v < step) {
        col = mix (col0, col1, v*steps);
    } else if (v >= step && v < 2.*step) {
        col = mix (col1, col2, (v - step)*steps);
    } else if (v >=  2.*step && v <  3.*step) {
        col = mix (col2, col3, (v -  2.*step)*steps);
    } else if (v >=  3.*step && v <  4.*step) {
        col = mix (col3, col4, (v -  3.*step)*steps);
    } else if (v >=  4.*step && v <  5.*step) {
        col = mix (col4, col5, (v -  4.*step)*steps);
    } else if (v >=  5.*step && v <  6.*step) {
        col = mix (col5, col6, (v -  5.*step)*steps);
    } else if (v >=  6.*step && v <  7.*step) {
        col = mix (col6, col7, (v -  6.*step)*steps);
    } else if (v >=  7.*step && v <  8.*step) {
        col = mix (col7, col8, (v -  7.*step)*steps);
    } else if (v >=  8.*step && v <  9.*step) {
        col = mix (col8, col9, (v -  8.*step)*steps);
    } else if (v >=  9.*step && v < 10.*step) {
        col = mix (col9, colA, (v -  9.*step)*steps);
    } else if (v >= 10.*step && v < 11.*step) {
        col = mix (colA, colB, (v - 10.*step)*steps);
    } else if (v >= 11.*step && v < 12.*step) {
        col = mix (colB, colC, (v - 11.*step)*steps);
    } else if (v >= 12.*step && v < 13.*step) {
        col = mix (colC, colD, (v - 12.*step)*steps);
    } else if (v >= 13.*step && v < 14.*step) {
        col = mix (colD, colE, (v - 13.*step)*steps);
    } else if (v >= 14.*step && v < 15.*step) {
        col = mix (colE, colF, (v - 14.*step)*steps);
    } else
        col = colF;

    return col;
}

mat3 r3d (float deg)
{
	float r = radians (deg);
    float c = cos (r);
    float s = sin (r);
    mat3 m = mat3 (vec3 ( c,  s, .0),
                   vec3 (-s,  c, .0),
                   vec3 (.0, .0, .0));

    return m;
}

float hash( float n ) {
    return fract(sin(n)*43758.5453);
}

float noise (in vec3 x)
{
    vec3 p = floor(x);
    vec3 f = fract(x);

    f = f*f*(3. - 2.*f);

    float n = p.x + p.y*57. + 113.*p.z;

    float res = mix(mix(mix( hash(n+  0.), hash(n+  1.),f.x),
                        mix( hash(n+ 57.), hash(n+ 58.),f.x),f.y),
                    mix(mix( hash(n+113.), hash(n+114.),f.x),
                        mix( hash(n+170.), hash(n+171.),f.x),f.y),f.z);
    return res;
}

float fbm (vec3 p)
{
    float f;
    mat3 m = r3d (1.1);

    f  = .5*noise( p ); p = m*p*2.02;
    f += .25*noise( p ); p = m*p*2.23;
    f += .125*noise( p ); p = m*p*2.71;
    f += .0625*noise( p );

    return f;
}

mat2 r2d (in float degree)
{
	float rad = radians (degree);
    float c = cos (rad);
    float s = sin (rad);
    return mat2 (c, s, -s, c);
}

// array is interpreted from 0..3 as:
//
//   - 0: bottom left value
//   - 1: bottom right value
//   - 2: top left value
//   - 3: top right value
//
float blerp (vec2 uv, float array[4])
{
    float m = smoothstep (.0, 1., uv.x);
    float a = mix (array[0], array[1], m);
    float b = mix (array[2], array[3], m);
    return mix (a, b, smoothstep (.0, 1., uv.y));
}

vec3 visualizeUV (vec2 uv)
{
	return vec3 (uv, .0);	
}

void main ()
{
    // normalize 'canvas'
    vec2 uv = fragCoord;
    uv *= 2. - 1.;
    uv.x *= iResolution.x/iResolution.y;

	uv -= .4*vec2 (cos (.5*iTime), sin (.75*iTime));

    // rotate 'canvas'
    uv *= r2d (24.*iTime);

    // scale 'canvas'
    float scale = 2. + 6.*(.5 + .5*cos (iTime));
    uv *= scale;

    vec2 grid = fract (uv);
    vec2 id = floor (uv);
    vec3 col = vec3 (.25);

    const vec2 offsets[4] = vec2[4](
        vec2(.0, .0),
        vec2(1., .0),
        vec2(.0, 1.),
        vec2(1., 1.));

    float array[4];
    
    for (int i = 0; i < 4; i++) {
        vec2 p = id + offsets[i];
        float v = fbm (vec3 (p, p.x - p.y));
        array[i] = v;
        
    }
	//array[0] = .1;
	//array[3] = .3;
	//array[2] = .6;
	//array[1] = .9;
    col = gradient (blerp (grid, array));
    //col = visualizeUV (grid);

    if (grid.x < .0125 || grid.y < .0125)
        col *= .85;

    fragColor = vec4 (col, 1.);
}
