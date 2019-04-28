#version 420
uniform vec3 iResolution;
uniform float iTime;
in vec2 fragCoord;
out vec4 fragColor;

precision highp float;

float hash1 (float v)
{
    return fract (sin (dot (vec2 (v),
                            vec2 (117.34324,
                                  23.94835))) * 45849.948593);
}

float hash2 (vec2 v)
{
    return fract (sin (dot (v,
                            vec2 (hash1 (v.x)*117.34324,
                                  hash1 (v.y)*23.94835))) * 45849.948593);

}

mat2 r2d (float deg)
{
    float rad = radians (deg);
    float c = cos (rad);
    float s = sin (rad);
    return mat2 (c, s, -s, c);
}

float map (vec3 p, out vec3 pout, out int id)
{
	p.z -= 4.*iTime;
    p.xy*=r2d (23.*iTime);
    float h = 2. + .5*cos(2.*iTime);

    float g = p.y + h + .4*(.5 + .5*cos(5.*iTime+p.z));
    g = min (g, -p.y + h + .2*(.5 + .5*cos(7.*iTime+p.z)));

    float w = p.x + 1.5*h + .5*(.5 + .5*sin(3.*iTime+p.z));
    w = min (w, -p.x + 1.5*h + .4*(.5 + .5*sin(4.*iTime+p.z)));

    float d = min (w, g);

    if (d == w) id = 1;
    if (d == g) id = 2;

    pout = p;

    return d;
}

float march (vec3 ro, vec3 rd, inout vec3 pout, inout int id)
{
    float t = .0;
    float d = .0;
    for (int i = 0; i < 32; ++i) {
        t = map (ro + d*rd, pout, id);
        if (abs (t) < .01*(1. + .125*t)) break;
        d += t*1.05;
    }
    return d;
}

vec3 normal (vec3 p)
{
    vec3 ignored;
    int ignored2;
    float d = map (p, ignored, ignored2);
    vec2 e = vec2 (.001, .0);
    return normalize (vec3 (map (p + e.xyy, ignored, ignored2),
                            map (p + e.yxy, ignored, ignored2),
                            map (p + e.yyx, ignored, ignored2)) - d);
}

float xor (float a, float b){return a*(1. - b) + b*(1. - a);}

vec3 shade (vec3 ro, vec3 rd, float d, vec3 pout, int id)
{
    vec3 col = vec3 (.0);
    float scale = 1.;
    vec2 cell = vec2 (.0);
    float density = .0;

    if (id == 1) {
        scale = 2.;
        cell = floor (scale*pout.yz);
        density = .65;
        float f = step (density, hash2 (cell));
        vec2 cc = scale*pout.yz - cell;
        float dd = length (cc*2.-1.);
        float m = smoothstep (.5, .55, xor (dd, 1./f));
		vec3 c = vec3 (hash1 (cell.x), .8, hash1 (cell.x+cell.y));
        col = mix (vec3 (.1), c, m);
    }
    if (id == 2) {
        scale = 8.;
        cell = floor (scale*pout.xz);
        density = .6;
        float f = step (density, hash2 (cell));
        vec2 cc = scale*pout.xz - cell;
        float dd = length (cc*2.-1.);
        float m = smoothstep (.5, .55, xor (dd, 1./f));
		vec3 c = vec3 (.9, hash1 (cell.x*cell.y), hash1 (cell.x));
        col = mix (vec3 (.1), c, m);
    }

    return col;
}

vec3 camera (vec2 uv, vec3 ro, vec3 aim, float zoom)
{
    vec3 f = normalize (aim - ro);
    vec3 wu = vec3 (.0, 1., .0);
    vec3 r = normalize (cross (wu, f));
    vec3 u = normalize (cross (f, r));
    vec3 c = ro + f*zoom;
    return normalize (c + uv.x*r + uv.y*u - ro);
}

void main ()
{
    vec2 uv = fragCoord*2. - 1.;
    uv.x *= iResolution.x/iResolution.y;

    vec3 ro = vec3 (.2*cos (iTime), .1*sin(iTime), 1.);
    vec3 aim = vec3 (.0);
    float zoom = 1.75;
    vec3 rd = camera (uv, ro, aim, zoom);
    vec3 pout;
    float fog;
    int id;

    float d = march (ro, rd, pout, id);
    fog = 1./exp(1. + d*d*.025);
    vec3 p = ro + d*rd;
    vec3 n = normal (p);
    vec3 col = shade (ro,rd, d, pout, id);

    ro = p + .01*n;
    rd = normalize (reflect (rd, n));
    d = march (ro, rd, pout, id);
    vec3 rcol = shade (ro, rd, d, pout, id);
    col += .1*rcol;

    col *= fog;
    col = col  / (.5 + col);
    col = pow (col, vec3 (1./2.2));

    fragColor = vec4 (col, 1.);
}
