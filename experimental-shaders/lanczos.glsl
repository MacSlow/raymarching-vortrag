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

#define Width 5.  // filter radius 
#define scale 92. // zoom
#define S     1.  // recons scale

#define sinc(x)    sin(PI*(x)) / (PI*(x))
#define Lanczos(x) sinc((x)/S) * sinc((x)/W)
#define PI         3.14159265359

vec4 T (vec2 uv)
{
    return texelFetch (iChannel0,
                       ivec2 (mod (uv, iChannelResolution0.y)),
                       0);
}

void main ()
{
    float W = Width;
    vec2 uv = fragCoord*iResolution.xy;
	uv /= scale;
    
    float t = 0.;
    for (float y = -W; y < W; y++) {         // convolution
        for (float x = -W; x < W; x++) {
            vec2 P = ceil(uv + vec2 (x,y));
            vec2 F = uv - P;
            vec2 I = Lanczos (F);            // reconstruction filter
            fragColor += I.x*I.y*(T (P) - .5);
            t += I.x*I.y;
        }
    }
    fragColor = .5 + 1.*fragColor/t;         // scale intensity
    //fragColor = sqrt (fragColor);            // to sRGB
    //fragColor = fragColor.rrrr;
}
