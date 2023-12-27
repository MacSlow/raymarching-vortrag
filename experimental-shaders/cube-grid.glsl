// build/raymarcher-gl cube-grid.glsl 640 360 30
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

const int MAX_ITER = 80;
const float EPSILON = .00025;
const float STEP_BIAS = .6;

float sdSphere (in vec3 p, in float r)
{
	return length (p) - r;
}

float sdPlane (in vec3 p, in float h)
{
	return p.y - h;
}

float udBox (in vec3 p, in vec3 size, in float r)
{
	return length (max (abs (p) - size, .0)) - r;
}

float opCombine (in float d1, in float d2, in float r, inout int matIndex)
{
    float h = clamp (.5 + .5 * (d2 - d1) / r, .0, 1.);
	float result = mix (d2, d1, h) - r * h * (1. - h);
	float threshold = r*.001;

	matIndex = 2;

	if (abs(result - d1) <= threshold) {
		matIndex = 0;
	}

	if (abs(result - d2) <= threshold) {
		matIndex = 1;
	}

    return result; 
}

vec2 opRepeat2 (inout vec2 p, in float size)
{
	float h = size*.5;
	vec2 cell = floor ((p + h) / size);
	p = mod (p + h, size) - h;
	return cell;
}

struct HitResult {
    float dist;
    int matId;
};

HitResult scene (in vec3 p)
{
	vec3 p2 = p + vec3 (-.1, -.3 - .1*(.5+.5*cos (3.*iTime)), .5);
	vec3 p3 = p + vec3 (.4, .0, 1.);
	float ball = sdSphere (p2, .1 + .1 * (.5+.5*cos(4.*iTime)));
	vec2 cell = opRepeat2 (p3.xz, .2);
	p3.y += .3*sin (.1*cell.x+1.5*iTime)*cos(.2*cell.y+2.*iTime);
	p3.y += .15*sin (.5*cell.y+3.*iTime)*cos(.5*cell.x+3.*iTime);
	float boxes = udBox (p3, vec3 (.075), .02);

    int matIndex = 0;
    float dist = opCombine (boxes, ball, .1, matIndex);
    int cellId = int (mod (floor (cell.y + cell.x), 2.)) + 1;
	int matIds[3] = int[3] (cellId, 0, 3);
    HitResult result = HitResult (dist, matIds[matIndex]);

    return result;
}

HitResult raymarch (in vec3 ro, in vec3 rd)
{
	float d = .0;
	float t = .0;
    HitResult result;
	for (int i = 0; i < MAX_ITER; ++i) {
		result = scene (ro + t*rd);
		d = result.dist;
		if (abs (d) < EPSILON*(1. + .125*d)) break;
		t += d*STEP_BIAS;
	}
    result.dist = t;
	return result;
}

vec3 normal (in vec3 p, in float epsilon)
{
	vec2 e = vec2 (epsilon, .0);
	float d = scene (p).dist;
	return normalize (vec3 (scene (p + e.xyy).dist,
							scene (p + e.yxy).dist,
							scene (p + e.yyx).dist) - d);
}

float shadow (in vec3 p, vec3 n, vec3 lPos)
{
	vec3 rd = normalize (lPos - p);
    float result = 1.;
    float t = .1;
	float ph = 1e10;
    for (int i = 0; i < 64; i++) {
        float h = scene (p + .01*n + t * rd).dist;
        if (h < .00001) return .0;
		float y = h*h/(2.*ph);
        float d = sqrt (h*h - y*y);
        result = min (result, 10.*d/max (.0, t - y));
        ph = h;
        t += h*.5;
    }

    return result;
}

vec3[4] materials = vec3[4] (vec3 (1.0, 1., 1.0),
                             vec3 (1., 1., 1.),
                             vec3 (.1, .1, .1),
                             vec3 (1., .0, .0));

vec3 shade (in vec3 ro, in vec3 rd, in float d, in vec3 n, in int matId)
{
	vec3 p = ro + d*rd;
	vec3 amb = vec3 (.1);
	vec3 diffC = vec3 (1., .5, .3);
	vec3 specC = vec3 (1., .95, .9);
	vec3 diffC2 = vec3 (.3, .5, 1.);
	vec3 specC2 = vec3 (.9, .95, 1.);

    vec3 lPos = ro + vec3 (.5, 1.0, -3.);
    vec3 lPos2 = ro + vec3 (-1., 1.2, 2.);
    vec3 lDir = lPos - p;
    vec3 lDir2 = lPos2 - p;
    vec3 lnDir = normalize (lDir);
    vec3 lnDir2 = normalize (lDir2);
    float sha = shadow (p, n, lPos);
    float sha2 = shadow (p, n, lPos2);
    float lDist = distance (p, lPos);
    float lDist2 = distance (p, lPos2);
    float attenuation = 8. / (lDist*lDist);
    float attenuation2 = 8. / (lDist2*lDist2);

    float diff = max (dot (n, lnDir), .0);
    float diff2 = max (dot (n, lnDir2), .0);
    vec3 h = normalize (lDir - rd);
    vec3 h2 = normalize (lDir2 - rd);
    float spec = pow (max (dot (h, n), .0), 20.);
    float spec2 = pow (max (dot (h2, n), .0), 40.);

    vec3 diffTerm = sha * attenuation * diff * diffC;
    vec3 diffTerm2 = sha2 * attenuation2 * diff2 * diffC2;
    vec3 specTerm = (sha > .1) ? attenuation * spec * specC : vec3 (.0);
    vec3 specTerm2 = (sha2 > .1) ? attenuation2 * spec2 * specC2 : vec3 (.0);

	vec3 color = materials[matId];
    return amb + diffTerm*color + specTerm + diffTerm2*color + specTerm2;
}   

vec3 camera (in vec2 uv, in vec3 ro, in float zoom, in vec3 aim)
{
	vec3 forward = normalize (aim - ro);
	vec3 worldUp = vec3 (.0, 1., .0);
	vec3 right = normalize (cross (forward, worldUp));
	vec3 up = normalize (cross (right, forward));
	vec3 center = normalize (ro + forward*zoom);
	return normalize ((center + uv.x*right + uv.y*up) - ro);
}

void main ()
{
    vec2 uv = fragCoord.xy;
	uv = uv * 2. - 1.;
	uv.x *= iResolution.x/iResolution.y;

	float offset = 1.7;
	vec3 ro = vec3 (offset*cos(.2*iTime), 1., offset*sin(.2*iTime));
	vec3 aim = vec3 (-offset*.05*cos (iTime),.1,-offset*.05*sin (iTime));
	float zoom = 2.;
	vec3 rd = camera (uv, ro, zoom, aim);

	HitResult result = raymarch (ro, rd);
	float d = result.dist;
	float fog = 1. / (1. + d*d*.2);
	vec3 p = ro + d*rd;
	vec3 n = normal (p, d*EPSILON);
	vec3 col = shade (ro, rd, d, n, result.matId);

	col *= fog;
	col = mix (col, vec3 (.65, .7, .9), pow (1. - 1. / d, 20.));
	col = col / (1. + col);
	col *= vec3 (.8, .7, .6);
	col = .2*col + .8*sqrt (col);

    fragColor = vec4 (col, 1.);
}
