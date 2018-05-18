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

const int MAX_ITER = 96;
const float STEP_SIZE = .75;
const float EPSILON = .001;
const float PI = 3.1415926;

struct Ray {
    vec3 ro;
    vec3 rd;
};

struct Result {
	vec3 point;
    vec3 normal;
    float dist;
    int iter;
    int id;
};

const Result nullResult = Result (vec3 (.0), vec3 (.0), .0, 0, -1);

float mandelbulb (vec3 pos, in float n, in float bail, in int iter)
{
	vec3 z = pos;
	float dr = 1.;
	float r = .0;
	for (int i = 0; i < iter; i++) {
		r = length (z);
		if (r > bail) break;
		
		// from cartesian to polar
		float theta = acos (z.z / r);
		float phi = atan (z.y, z.x);
		dr = pow(r, n - 1.) * n * dr + 1.;
		
		// scale and rotate the point
		float zr = pow (r, n);
		theta = theta * n;
		phi = phi * n;
		
		// back to cartesian
		z = zr * vec3(sin(theta)*cos(phi), sin(theta)*sin(phi), cos(theta));
		z += pos;
	}

    return .5 * log (r) * r / dr; // I just don't get this distance estimator here
}

float map (in vec3 p)
{
	float d = mandelbulb (p, 8., 4., 8);
    return d;
}

vec3 normal (in vec3 p)
{
    float d = map (p);
    vec3 e = vec3 (.001, .0, .0);
    vec3 n = vec3 (map (p + e.xyy) - d,
                   map (p + e.yxy) - d,
                   map (p + e.yyx) - d);

    return normalize (n);
}

Result raymarch (in Ray ray)
{
    Result res = nullResult;

    for (int i = 0; i < MAX_ITER; ++i) {
        res.iter = i;
        float tmp = map (ray.ro + res.dist * ray.rd);
        if (tmp < EPSILON) break;
        res.dist += tmp * STEP_SIZE;
    }

    res.point = ray.ro + res.dist * ray.rd;
    res.normal = normal (res.point);
    //res.id = tmp.id;

    return res;
}

float shadow (in Ray ray, in vec3 lPos)
{
    float distToLight = distance (lPos, ray.ro);
    float dist = .0;

    for (int i = 0; i < MAX_ITER; ++i) {
        float tmp = map (ray.ro + dist * ray.rd);
        if (tmp < EPSILON) {
            if (dist < distToLight)
                return .125;
            else
                return 1.;
        }
        dist += tmp * STEP_SIZE;
    }

    return 1.;
}

vec3 shade (in Result res)
{    
    vec3 amb = vec3 (.1);
    vec3 diffM = vec3 (.9, .25, .2);
    vec3 diffL = vec3 (.95, .9, .3)*3.;
    vec3 diffL2 = vec3 (.5, .7, .85)*2.;
    vec3 specM = vec3 (1.);
    vec3 specL = vec3 (1.);
    vec3 lPos = vec3 (.4, 1., -1.25);
    vec3 lPos2 = vec3 (-.25, .5, 1.);
	vec3 lDir = normalize (lPos - res.point);
	vec3 lDir2 = normalize (lPos2 - res.point);

    float diff = clamp (dot (res.normal, lDir), .0, 1.);
    float diff2 = clamp (dot (res.normal, lDir2), .0, 1.);
    vec3 h = normalize (lDir + res.point);
    vec3 h2 = normalize (lDir2 + res.point);
    float shininess = 60.;
    float spec = pow (clamp (dot (res.normal, h), .0, 1.), shininess);
    float spec2 = pow (clamp (dot (res.normal, h2), .0, 1.), shininess);
    float sha = shadow (Ray (res.point + .001 * res.normal, lDir), lPos);
    float sha2 = shadow (Ray (res.point + .001 * res.normal, lDir2), lPos2);

    return amb + sha * (diff * diffM * diffL + spec * specM * specL) + 
        sha2 * (diff2 * diffM * diffL2 + spec2 * specM * specL);
}

void main ()
{
	vec2 uv = fragCoord.xy;
    uv = uv * 2. - 1.;
    uv.x *= iResolution.x / iResolution.y;

    float r = 1.5 + .5 * (cos (iTime)*.5+.5);
    float theta = (iMouse.x / iResolution.x) * PI;
    float phi = -(iMouse.y / iResolution.y * 2. - 1.) * PI;
    float x = r * sin (theta) * cos (phi);
    float y = r * sin (theta) * sin (phi);
	float z = r * cos (theta);
    //vec3 ro = vec3 (r*cos(iTime), cos(1.3*iTime), r*sin(iTime));
    vec3 ro = vec3 (x, y, z);
    float zoom = 2.;
    vec3 lookAt = vec3 (.0);
    vec3 forward = normalize (lookAt - ro);
    vec3 worldUp = vec3 (.0, 1., .0);
    vec3 right = normalize (cross (worldUp, forward));
    vec3 up = normalize (cross (forward, right));
    vec3 camCenter = ro + zoom * forward;
    vec3 i = camCenter + uv.x * right + uv.y * up;
    vec3 rd = normalize (i - ro);

    Ray ray = Ray (ro, rd);
    Result res = raymarch (ray);
    vec3 col = shade (res);
    float fog = float (res.iter) / float (MAX_ITER);
    col *= 1. - (fog * fog);

    col = col / (.85 + col);
    col *= vec3 (.95, .9, .85);
    col = .3*col + .7*sqrt (col);

    fragColor = vec4 (col, 1.);
}
