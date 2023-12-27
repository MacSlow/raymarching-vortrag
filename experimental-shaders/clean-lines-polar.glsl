#version 420
uniform vec3 iResolution;
uniform float iTime;
uniform vec3 iChannelResolution0;
uniform vec3 iChannelResolution1;
uniform vec3 iChannelResolution2;
uniform vec3 iChannelResolution3;
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

mat2 r2d (in float degree)
{
    float rad = radians (degree);
    float c = cos (rad);
    float s = sin (rad);
	return mat2 (c, s, -s, c);
}

vec2 cart2polar (in vec2 cart)
{
    float r = length (cart);
    float phi = atan (cart.x, cart.y);
    return vec2 (phi, r);
}

vec2 polar2cart (in vec2 polar)
{
    float x = polar.x*cos (polar.y);
    float y = polar.x*sin (polar.y);
    return vec2 (y, x);
}

float graph (in vec2 p)
{
    vec2 polar = cart2polar (p);
    polar.y = abs (4.*sin (.75*polar.y) * cos (2.*polar.x));
    p = polar2cart (polar);

	return length (p.x / p.y);
}

vec2 gradient (in vec2 p)
{
    //return vec2 (dFdx (graph (p)), dFdy (graph (p))) / fwidth (graph (p));
    vec2 e = vec2 (.001, .0);
    return vec2 (graph (p - e.xy) - graph (p + e.xy),
                 graph (p - e.yx) - graph (p + e.yx)) / (.75*e.x);
}

void main ()
{
    vec2 uv = fragCoord;
    uv = uv * 2. - 1.;
    uv.x *= iResolution.x / iResolution.y;

    vec2 p = (5. + 10.*(.5 + .5*cos(3.*iTime)))*uv;
    p *= r2d (34.*iTime);
    p += vec2 (3.*cos (iTime), 3.*sin (iTime));
    p.x += .5*cos (p.y + 2.*iTime);
    p.y += .5*sin (p.x + 4.*iTime);

    float d = abs (graph (p)) / length (gradient (p));
    float thickness = 20. / iResolution.y;
    vec3 col = vec3 (smoothstep (1.*thickness, 2.*thickness, d));

	vec3 c = vec3 (1., .95, .9);
	c *= .8 + .2*smoothstep (.0, 25.*(2./iResolution.y), d);
	col = mix (c, col, 1. - smoothstep (.0, 12.*(1./iResolution.y), d));

    col *= 1. - .4*length (fragCoord/iResolution.xy*2. - 1.);
    col = col / (1. + col);
    col = pow (col, vec3 (1./2.2));
    fragColor = vec4 (col, 1.);
}
