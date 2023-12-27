#version 420
uniform vec3 iResolution;
uniform float iTime;
uniform float iFrameRate;
uniform int  iFrame;
uniform vec4 iMouse;
in vec2 fragCoord;
out vec4 fragColor;

precision highp float;

mat2 r2d (float deg)
{
    float r = radians (deg);
    float c = cos (r);
    float s = sin (r);
    return mat2 (c, s, -s, c);
}

float line (vec2 uv, vec2 a, vec2 b, float thickness)
{
    vec2 pa = uv - a;
    vec2 ba = b - a;
    float h = clamp (dot (pa, ba) / dot (ba, ba), .0, 1.);
    float d = length (pa - ba * h);
    return smoothstep (.0, .0075, d);
}

float arrow (vec2 uv, vec2 a, vec2 b, float thickness)
{
	float m = line (uv, a, b, thickness);
	vec2 vector = b - a;
	float len = length (vector);
	vec2 tipends = .05*len*normalize (vec2 (vector.y, -vector.x));
	m *= line (uv, b, b - .1*vector + tipends, thickness);
	m *= line (uv, b, b - .1*vector - tipends, thickness);

    return m;
}

float point (vec2 uv, vec2 c, float r)
{
    float d = length (uv - c) - r;
    return smoothstep (.0, .01, d);
}

bool intersectsTriangle (vec2[3] triangle, vec2 p, out float w1, out float w2)
{
    vec2 a = triangle[0];
    vec2 b = triangle[1];
    vec2 c = triangle[2];
    vec2 ab = b - a;
    vec2 ac = c - a;

    w2 = (p.y*ab.x - a.y*ab.x - p.x*ab.y + a.x*ab.y)/(ac.y*ab.x - ac.x*ab.y);
    w1 = (p.x - a.x - w2*ac.x)/ab.x;

    if (w1 >= .0 && w2 >= .0 && (w1 + w2 <= 1.)) {
        return true;
    }

    return false;
}

void main ()
{
    vec2 uv = fragCoord;
	float aspect = iResolution.x/iResolution.y;
    uv = uv*2. - 1.;
    uv.x *= aspect;

    float pointSize = .01;
    mat2 rotation = r2d (iTime*25.);
    vec2 a = vec2 (.1, .6)*rotation;
    vec2 b = vec2 (-.4, -.2)*rotation;
    vec2 c = vec2 (.7, -.4)*rotation;
    float m = line (uv, a, b, pointSize);
    m *= line (uv, b, c, pointSize);
    m *= line (uv, c, a, pointSize);

    vec2 p;
	p.x = aspect*(iMouse.x/iResolution.x*2. - 1.);
	p.y = -(iMouse.y/iResolution.y*2. - 1.);

    vec2[3] triangle = vec2[3] (a, b, c);
	float w1 = .0;
	float w2 = .0;
    bool isInside = intersectsTriangle (triangle, p, w1, w2);
    vec3 colP = (isInside ? vec3 (.0, 1., .0) : vec3 (1., .0, .0))*(1. - point (uv, p, pointSize));

	float n = .0;
	vec2 ab = b - a;
	vec2 ac = c - a;
	n = arrow (uv, a, a + w1*ab, pointSize);
	n *= arrow (uv, a + w1*ab, a + w1*ab + w2*ac, pointSize);

    vec3 col = vec3 (.5)*(1. - m) + colP + (isInside ? (1. - n)*vec3(1., .5, .25) : (1. - n)*vec3 (.25, .5, 1.));
    fragColor = vec4 (col, 1.);
}

