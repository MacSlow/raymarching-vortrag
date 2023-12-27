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

const vec3 col1 = vec3 (6./255., 23./255., 102./255.);
const vec3 col3 = vec3 (76./255., 12./255., 130./255.);
const vec3 col5 = vec3 (194./255., 1./255., 101./255.);
const vec3 col7 = vec3 (217./255., 59./255., 10./255.);
const vec3 col9 = vec3 (255./255., 156./255., .0/255.);
const vec3 col2 = mix (col1, col3, .5);
const vec3 col4 = mix (col3, col5, .5);
const vec3 col6 = mix (col5, col7, .5);
const vec3 col8 = mix (col7, col9, .5);
const vec3 colA = mix (col9, col1, .5);

vec3 gradient (float v) {
    float steps = 10.;
    float step = 1. / steps;
    vec3 col = vec3 (.4);

    if (v >= .0 && v < step) {
        col = mix (col1, col2, v * steps);
    } else if (v >= step && v < 2.0 * step) {
        col = mix (col2, col3, (v - step) * steps);
    } else if (v >= 2.0 * step && v < 3.0 * step) {
        col = mix (col3, col4, (v - 2.0 * step) * steps);
    } else if (v >= 3.0 * step && v < 4.0 * step) {
        col = mix (col4, col5, (v - 3.0 * step) * steps);
    } else if (v >= 4.0 * step && v < 5.0 * step) {
        col = mix (col5, col6, (v - 4.0 * step) * steps);
    } else if (v >= 5.0 * step && v < 6.0 * step) {
        col = mix (col6, col7, (v - 5.0 * step) * steps);
    } else if (v >= 6.0 * step && v < 7.0 * step) {
        col = mix (col7, col8, (v - 6.0 * step) * steps);
    } else if (v >= 7.0 * step && v < 8.0 * step) {
        col = mix (col8, col9, (v - 7.0 * step) * steps);
    } else if (v >= 8.0 * step && v < 9.0 * step) {
        col = mix (col9, colA, (v - 8.0 * step) * steps);
    }

    return col;
}

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
	float time = iTime + 5.;

    // normalize and aspect-correct UVs
    vec2 uv = fragCoord*2. - 1.;
    vec2 uvRaw = uv;
	float aspect = iResolution.x/iResolution.y;
    uv.x *= aspect;

    // rotate and move the canvas
    uv *= r2d (24.*time);
    uv += vec2 (.2*cos (time), .3*sin(time));

    // cause the canvas to 'zoom'
    float scale = 6. + 3.*(.5 + .5*cos(time));
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
            vec2 p = offset + .5*vec2 (cos (3.*sin(.005*time)*n.x*(time + 22.75)), sin (2.*cos(.007*time)*n.y*(time + 22.75)));
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
    float colorOne = mix (brightColor, darkColor, .25);
    vec3 colorTwo;

    if (uvRaw.x < sin (iTime + 6.25)) {
        colorTwo = mix (gradient (flatColor), vec3 (invertedFlatColor), .035);
    } else {
        colorTwo = mix (vec3 (flatColor), vec3 (invertedFlatColor), .035);
    }

    vec3 colorFinal = mix (vec3 (colorOne), vec3 (colorTwo), .9);

    // gamma-correct
    colorFinal = pow (colorFinal, vec3 (1./2.2));

    fragColor = vec4 (colorFinal, 1.);
}

