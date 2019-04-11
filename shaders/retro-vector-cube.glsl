#version 420

uniform vec3 iResolution;
uniform float iTime;
uniform vec4 iMouse;
in vec2 fragCoord;
out vec4 fragColor;

const vec3 red = vec3 (1.0, 0.0, 0.0);
const vec3 green = vec3 (0.0, 1.0, 0.0);
const vec3 blue = vec3 (0.0, 0.0, 1.0);
const vec3 white = vec3 (1.0);
const vec3 orange = vec3 (1.0, 0.4, 0.125);
const vec3 black = vec3 (0.2, 0.3, 0.2);
const vec3 cyan = vec3 (0.0, 1.0, 1.0);
const vec3 magenta = vec3 (1.0, 0.0, 1.0);
const vec3 yellow = vec3 (1.0, 1.0, 0.0);
const float SIZE = .003;

float distLine (vec2 p, vec2 a, vec2 b) {
    vec2 pa = p - a;
    vec2 ba = b - a;
    float t = clamp ( dot (pa, ba) / dot (ba, ba), .0, 1.);
    return length (pa - ba*t);
}

float lineMask (vec2 uv, vec2 a, vec2 b) {
    float d = distLine (uv, a, b);
    float thickness = SIZE;
    return smoothstep (thickness, .125*thickness, d);
}

vec3 glowLine (vec2 uv, vec2 a, vec2 b, vec3 rgbGlow) {
    float m = lineMask (uv, a, b);
    float dist = distLine (uv, a, b);
    float brightness = SIZE/pow (.085 + 2.*dist, 2.);
    vec3 color = m*vec3 (.7);
    color += rgbGlow*brightness;
	return color;
}

struct boxType {vec4 p[8];};

mat4 trans (vec3 t)
{
    mat4 mat = mat4 (vec4 (1., .0, .0, .0),
                     vec4 (.0, 1., .0, .0),
                     vec4 (.0, .0, 1., .0),
                     vec4 (t.x, t.y, t.z, 1.));
    return mat;
}

mat4 rotX (float angle)
{
    float rad = radians (angle);
    float c = cos (rad);
    float s = sin (rad);

    mat4 mat = mat4 (vec4 (1., .0, .0, .0),
                     vec4 (.0,   c,   s, .0),
                     vec4 (.0,  -s,   c, .0),
                     vec4 (.0, .0, .0, 1.));

    return mat;
}

mat4 rotY (float angle)
{
    float rad = radians (angle);
    float c = cos (rad);
    float s = sin (rad);

    mat4 mat = mat4 (vec4 (  c, .0,  -s, .0),
                     vec4 (.0, 1., .0, .0),
                     vec4 (  s, .0,   c, .0),
                     vec4 (.0, .0, .0, 1.));

    return mat;
}

mat4 rotZ (float angle)
{
    float rad = radians (angle);
    float c = cos (rad);
    float s = sin (rad);

    mat4 mat = mat4 (vec4 (  c,   s, .0, .0),
                     vec4 ( -s,   c, .0, .0),
                     vec4 (.0, .0, 1.0, .0),
                     vec4 (.0, .0, .0, 1.));

    return mat;
}

void main() {
    vec2 uv = fragCoord.xy* 2. - 1.;
    uv.x *= iResolution.x/iResolution.y;
    uv *= 1. + .4*length(uv);

    boxType box;
    box.p[0] = vec4 ( 0.1,  0.1,  0.1, 1.0);
    box.p[1] = vec4 ( 0.1, -0.1,  0.1, 1.0);
    box.p[2] = vec4 (-0.1, -0.1,  0.1, 1.0);
    box.p[3] = vec4 (-0.1,  0.1,  0.1, 1.0);
    box.p[4] = vec4 ( 0.1,  0.1, -0.1, 1.0);
    box.p[5] = vec4 ( 0.1, -0.1, -0.1, 1.0);
    box.p[6] = vec4 (-0.1, -0.1, -0.1, 1.0);
    box.p[7] = vec4 (-0.1,  0.1, -0.1, 1.0);

    float t = 8. + 14.*iTime;
    mat4 rot3d = rotX (-4.*t)*rotY (3.*t)*rotZ (2.*t);
    mat4 model = trans (vec3 (.0, .0, -.275))*rot3d;
    box.p[0] = model * box.p[0];
    box.p[1] = model * box.p[1];
    box.p[2] = model * box.p[2];
    box.p[3] = model * box.p[3];
    box.p[4] = model * box.p[4];
    box.p[5] = model * box.p[5];
    box.p[6] = model * box.p[6];
    box.p[7] = model * box.p[7];

    vec3 boxCol = glowLine (uv, box.p[0].xy / box.p[0].z, box.p[1].xy / box.p[1].z, red);
    boxCol += glowLine (uv, box.p[1].xy / box.p[1].z, box.p[2].xy / box.p[2].z, green);
    boxCol += glowLine (uv, box.p[2].xy / box.p[2].z, box.p[3].xy / box.p[3].z, orange);
    boxCol += glowLine (uv, box.p[3].xy / box.p[3].z, box.p[0].xy / box.p[0].z, cyan);
    boxCol += glowLine (uv, box.p[4].xy / box.p[4].z, box.p[5].xy / box.p[5].z, blue);
    boxCol += glowLine (uv, box.p[5].xy / box.p[5].z, box.p[6].xy / box.p[6].z, red);
    boxCol += glowLine (uv, box.p[6].xy / box.p[6].z, box.p[7].xy / box.p[7].z, yellow);
    boxCol += glowLine (uv, box.p[7].xy / box.p[7].z, box.p[4].xy / box.p[4].z, green);
    boxCol += glowLine (uv, box.p[0].xy / box.p[0].z, box.p[4].xy / box.p[4].z, blue);
    boxCol += glowLine (uv, box.p[1].xy / box.p[1].z, box.p[5].xy / box.p[5].z, cyan);
    boxCol += glowLine (uv, box.p[2].xy / box.p[2].z, box.p[6].xy / box.p[6].z, green);
    boxCol += glowLine (uv, box.p[3].xy / box.p[3].z, box.p[7].xy / box.p[7].z, magenta);

    boxCol = boxCol / (1. + boxCol);
    boxCol = sqrt (boxCol);
    boxCol *= mix (1., .5, .5 + .5*cos (500.*uv.y));

    fragColor = vec4(boxCol, 1.);
}
