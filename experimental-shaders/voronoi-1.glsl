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

vec2 noise2d (in vec2 p)
{
	vec3 v = fract (p.xyx*vec3(123.34, 234.34, 345.65));
	v += dot (v, v + 34.45);
	return fract (vec2 (v.x*v.y, v.y*v.z));
}

mat2 r2d (in float degree)
{
	float rad = radians (degree);
    float c = cos (rad);
    float s = sin (rad);
    return mat2 (c, s, -s, c);
}

void main ()
{
	float time = iTime + 3.;

    // normalize and aspect-correct UVs
    vec2 uv = fragCoord*2. - 1.;
	float aspect = iResolution.x/iResolution.y;
    uv.x *= aspect;

    // rotate and move the canvas
    uv *= r2d (24.*time);
    uv += vec2 (.2*cos (time), .3*sin(time));

    // cause the canvas to 'zoom'
    float scale = 3. + 3.*(.5 + .5*cos(time));
    uv *= scale;
    float rowSize = scale *.5;

    // split canvas up
    vec2 gv = fract (uv) - .5;
    vec2 id = floor (uv);

    float cellIndex = .0; 
    float minDist = 2.;

    // check only the adjacent grid-cells
    for (float y = -1.; y <= 1.; ++y) {
        for (float x = -1.; x <= 1.; ++x) {
            vec2 offset = vec2 (x, y); 
            vec2 n = noise2d (id + offset);
            vec2 p = offset + .5*sin (n*(time + 22.75));
            float d = dot (gv - p, gv - p);
            if (d < minDist) {
                minDist = d;
                cellIndex = (id.y + y) * rowSize + (id.x + x); 
            }
        }
    }   

    // 'color' cells in different styles
    float brightColor = (1. - minDist)*(1. - minDist);
    float darkColor = minDist*minDist;
    float flatColor = abs (cellIndex) / (scale*scale);
    float invertedFlatColor = 1. - flatColor;

    // blend between the four different style-variations
    float colorOne = mix (brightColor, darkColor, (.5 + .5*cos(1.75*time)));
    float colorTwo = mix (flatColor, invertedFlatColor, (.5 + .5*cos(1.75*time)));
    vec3 colorFinal = vec3 (mix (colorOne, colorTwo, .5 + .5*cos(.35*time)));

    // tone-map, gamma-correct
    //colorFinal = colorFinal / (1. + colorFinal);
    colorFinal = pow (colorFinal, vec3 (1./2.2));

    fragColor = vec4 (colorFinal, 1.);
}
