#version 420
uniform vec3 iResolution;
uniform float iTime;
uniform float iTimeDelta;
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

float opRepeat (inout float v, in float size)
{
	float halfSize = .5*size;
	float cell = floor ((v + halfSize) / size);
	v = mod (v + halfSize, size) - halfSize;
	return cell;
}

mat2 r2d (in float degree)
{
	float rad = radians (degree);
	float c = cos (rad);
	float s = sin (rad);
	return mat2(c, s, -s, c);
}

float scene (in vec2 p)
{
	float t = iTime;
	vec2 pt = 10.*p;
	float cellIndexY = opRepeat (pt.y, 5.);
	float sinus = length (sin (pt.x - cellIndexY*cos (5.*t)) - pt.y);
	float cellIndexX = opRepeat (pt.x, 4.);
	float circles = length (pt + .3*vec2 (cellIndexX, cellIndexY)*sin (5.*t)) - (.3 + .2*(.5 + .5*cos(3.*t)));

	return min (sinus, circles);
}

vec2 gradient (in vec2 p)
{
	vec2 e = vec2 (.001, .0);
	return vec2 (scene (p + e.xy) - scene (p - e.xy),
				 scene (p + e.yx) - scene (p - e.yx)) / (4.*e.x);
}

void main ()
{
    // normalize and aspect-correct
    vec2 uv = fragCoord.xy;
    uv = uv * 2. - 1.;
    uv.x *= iResolution.x/iResolution.y;

	// get the distance to the actual line of the function
	vec2 pt = uv;
	float dist = abs (scene (pt)) / length (gradient (pt));

	// base "paper"-color and drop-shadows
	vec3 base = vec3 (1., .95, .9);
	base *= .8 + .2*smoothstep (.0, 25.*(2./iResolution.y), dist);

	// mix solid black lines with drop-shadow
	float thickness = 4.;
	vec3 color = mix (base, vec3 (.0), 1. - smoothstep (.0, thickness*(2./iResolution.y), dist));

	// slight vignette
	color *= 1. - .6*length (fragCoord*2. - 1.);

    // tone-map, gamma-correct
    color = color / (1. + color);
    color = pow (color, vec3 (1./2.2));

    fragColor = vec4 (color, 1.);
}
