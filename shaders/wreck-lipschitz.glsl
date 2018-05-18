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

float udBox (in vec3 p, in vec3 size, in float r)
{
    return length (max (abs (p) - (size -r), .0)) - r;
}

float sdBox (in vec3 p, in vec3 size, in float r)
{
  vec3 d = abs(p) - size;
  return min (max (d.x, max (d.y,d.z)), .0) + length (max (d, .0)) - r;
}

float sdSphere (in vec3 p, in float r)
{
    return length (p) - r;
}

vec3 opRepeat3 (in vec3 p, float d)
{
    return mod (p + d*.5, d) - d*.5;
}

float opUnion (in float a, in float b)
{
	return min (a, b);
}

float opIntersect (in float a, in float b)
{
	return max (a, b);
}

float opSubtract (in float a, in float b)
{
	return max (-a, b);
}

float opBend (inout vec3 p, float deg)
{
    float rad = radians (deg);
    float cy = cos (rad * p.y);
    float sy = sin (rad * p.y);
    mat2  my = mat2 (cy, -sy, sy, cy);
    float cx = cos (rad * p.x);
    float sx = sin (rad * p.x);
    mat2  mx = mat2 (cx, -sx, sx, cx);
    p = mix (vec3 (mx* p.zx, p.y), vec3 (my * p.xy, p.z), .15*cos (p.x));

    return .0;
}

float displace (vec3 p)
{
    float result = 1.;
    float factor = 6. + 4.*cos (2.*iTime);
	result = .375 * sin (factor * p.x) * cos (factor * p.y) * sin (factor * p.z);

    return result;
}

mat2 r2d (in float degree)
{
	float rad = radians (degree);
    float c = cos (rad);
    float s = sin (rad);

    return mat2 (c, s, -s, c);
}

float scene (in vec3 p)
{
    vec2 offset = .75 * vec2 (cos (6.28+iTime), sin (6.28+iTime));

    vec3 w = (p + vec3 (-offset.x, -.125+offset.y, .0));
    opBend (w, 45. * cos (.25*iTime));
    w.zx *= r2d (50.*iTime);
    w.xy *= r2d (-75.*iTime);
    float dt = sdSphere (w, .65);
    float dp = displace (w);
    float thing = dt + dp; 
    
    p.xz *= r2d (-12.*iTime);
    p.yx *= r2d (8.*iTime);
    vec3 p1 = p + vec3 (.355);
    p1 = opRepeat3 (p1, .325);
    float boxes = udBox (p1, vec3 (.15), .03);
    float cutBox = sdBox (p, vec3 (2.7), .05);
    float wallBox = sdBox (p, vec3 (2.8), .05);

    return opUnion (thing, opUnion (-wallBox, opSubtract (cutBox, boxes)));
}

float raymarch (in vec3 ro, in vec3 rd)
{
    float t = .0;
    float d = .0;
    for (int i = 0; i < 96; ++i) {
        vec3 p = ro + d * rd;
        t = scene (p);
        if (abs(t) < .001*(1. + .125*t)) break;
        d += t * .5;
    }

    return d;
}

vec3 normal (in vec3 p)
{
    vec2 e = vec2 (.0002, .0);
    return normalize (vec3 (scene (p + e.xyy),
                            scene (p + e.yxy),
                            scene (p + e.yyx)) - scene (p));
}

vec3 shade (in vec3 ro, in vec3 rd, in float d)
{
    vec3 p = ro + d * rd;
    vec3 ambient = vec3 (.05);
    vec3 diffuseColor = vec3 (.9, .3, .3);
    vec3 specularColor = vec3 (.9, .8, .7);
    float shininess = 40.;
    float diffuseStrength = .25;
    float t = 3.*iTime;

    vec3 n = normal (p);
    vec3 lPos = 1.5 * vec3 (cos (t), 1., sin (t));
    float lDist = distance (lPos, p);
    vec3 lDir = normalize (lPos - p);
    vec3 hDir = normalize (ro + lDir);
    float diffuse = max (dot (n, lDir), .0)*(1. / lDist)*diffuseStrength;
    float specular = pow (max (dot (hDir, n), .0), shininess);

	vec3 diffuseColor2 = vec3 (.3, .9, .3);
    vec3 specularColor2 = vec3 (.7, .8, .9);
    vec3 lPos2 = -1.5 * vec3 (.0, sin(t), .75*cos(t));
    float lDist2 = distance (lPos2, p);
    vec3 lDir2 = normalize (lPos2 - p);
    vec3 hDir2 = normalize (ro + lDir2);
    float diffuse2 = max (dot (n, lDir2), .0)*(1. / lDist2)*diffuseStrength;
    float specular2 = pow (max (dot (hDir2, n), .0), shininess);

	vec3 diffuseColor3 = vec3 (.3, .3, .9);
    vec3 specularColor3 = vec3 (.8, .9, .7);
    vec3 lPos3 = 1.5 * vec3 (sin (t), .5*cos(t), -1.);
    float lDist3 = distance (lPos3, p);
    vec3 lDir3 = normalize (lPos3 - p);
    vec3 hDir3 = normalize (ro + lDir3);
    float diffuse3 = max (dot (n, lDir3), .0)*(1. / lDist3)*diffuseStrength;
    float specular3 = pow (max (dot (hDir3, n), .0), shininess);

    vec3 col = ambient +
			   diffuse * diffuseColor + specular * specularColor +
			   diffuse2 * diffuseColor2 + specular2 * specularColor2 +
			   diffuse3 * diffuseColor3 + specular3 * specularColor3;
    return col;
}

vec3 camera (in vec2 uv, in vec3 ro, in vec3 aim, in float zoom)
{
    vec3 camForward = normalize (vec3 (aim - ro));
    vec3 worldUp = vec3 (.0, 1., .0);
    vec3 camRight = normalize (cross (worldUp, camForward));
    vec3 camUp = normalize (cross (camForward, camRight));
    vec3 camCenter = ro + camForward * zoom;
    
    return normalize (camCenter + uv.x * camRight + uv.y * camUp - ro);
}

void main ()
{
    vec2 uv = fragCoord.xy;
    vec2 uvRaw = uv;
    uv = uv * 2. - 1.;
    uv.x *= iResolution.x / iResolution.y;

    // set up "camera", view origin (ro) and view direction (rd)
    float angle = radians (300. + 55. * iTime);
    float dist = 2.6;
    vec3 ro = vec3 (dist * cos (angle), cos (iTime), dist * sin (angle));
    vec3 aim = vec3 (.0);
    float zoom = 2.;
    vec3 rd = camera (uv, ro, aim, zoom);

    // primary-/view-ray
    float d = raymarch (ro, rd);
    vec3 p = ro + d * rd;
    vec3 n = normal (p);
    vec3 col = shade (ro, rd, d);
    col = mix (col, vec3 (.95, .85, .7), pow (1. - 1. / d, 10.));

    // secondary-/1st reflection-ray
    vec3 rd2 = normalize (reflect (rd, n));
    float d2 = raymarch (p + n*.001, rd2);
    vec3 p2 = p + d2 * rd2;
    vec3 n2 = normal (p2);
    vec3 col2 = shade (p, rd2, d2);
    col += (.1 + .05*(.5 + .5 * cos (5.*iTime))) * col2;

    // ternary-/2nd reflection-ray
    vec3 rd3 = normalize (reflect (rd2, n2));
    float d3 = raymarch (p2 + n2*.001, rd3);
    vec3 p3 = p2 + d3 * rd3;
    vec3 n3 = normal (p3);
    vec3 col3 = shade (p, rd3, d3);
    col += (.05 + .025*(.5 + .5 * cos (5.*iTime))) * col3;

    // tint, tone-mapping, gamma-correction, vingette
    col *= vec3 (.85, .9, .95);
    col = col / (.85 + col);
    col = .1 * col + .9 * sqrt (col);
    col *= .2 + .8 * pow (16. * uvRaw.x * uvRaw.y * (1. - uvRaw.x) * (1. - uvRaw.y), .3);

    fragColor = vec4(col, 1.);
}
