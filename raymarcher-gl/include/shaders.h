////////////////////////////////////////////////////////////////////////////////
//
// A desktop-based GLSL-environment similar to the WebGL-based site
// www.shadertoy.com
//
// Copyright 2015-2016 Mirco Müller
//
// Author(s):
//   Mirco "MacSlow" Müller <macslow@gmail.com>
//
// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License version 3, as published
// by the Free Software Foundation.
//
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranties of
// MERCHANTABILITY, SATISFACTORY QUALITY, or FITNESS FOR A PARTICULAR
// PURPOSE.  See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program.  If not, see <http://www.gnu.org/licenses/>.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef _SHADERS_H
#define _SHADERS_H

#define GLSL(src) "#version 130\n" #src

const char vert[] = GLSL(
    attribute vec2 aPosition;
    attribute vec2 aTexCoord;
    out vec2 fragCoord;
    void main()
    {
        gl_Position = vec4 (aPosition, -1.0, 1.0);
        fragCoord = aTexCoord;
    }
);

const char frag3DMetaBalls[] = GLSL(
    // the uniform inputs taken over from shadertoy.com
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

/*
 * ShaderToy uniforms not yet implemented:
 *   - uniform float iChannelTime[4]; // channel playback time (in seconds)
 *   - uniform float iSampleRate;     // sound sample rate (i.e., 44100)
 */

	precision highp float;

	float fbm (in vec2 p) {
		float v = 1.;//texture (iChannel0, p).r;
	    return v;
	}

	mat2 r2d (in float a) {
		float c = cos (radians (a));
	    float s = sin (radians (a));
	    return mat2 (vec2(c, s), vec2(-s, c));
	}

	vec2 opRepeat2 (inout vec2 p, in vec2 size) {
	    vec2 hsize = .5 * size;
	    vec2 cell = floor ((p + hsize) / size);
	    p = mod (p + hsize, size) - hsize;
	    return cell;
	}

	float opCombine (in float d1, in float d2, in float r)
	{
	    float h = clamp (.5 + .5 * (d2 - d1) / r, .0, 1.);
	    return mix (d2, d1, h) - r * h * (1. - h);
	}

	float sdSphere (in vec3 p, in float r) {
	    return length (p) - r;
	}

	float sdHexPrism (in vec3 p, in vec2 h) {
	    vec3 q = abs (p);
	    return max (q.z - h.y, max ((q.x * .866025 + q.y * .5), q.y) - h.x);
	}

	float sdThing (in vec3 p) {
	    p.xz *= r2d(20.*iTime);
	    p.yz *= r2d(-40.*iTime);
	    float r = 1.25 + .75 * (.5 + .5 * cos (p.y * 7.))*
	                     .75 * (.5 + .5 * cos (p.x * 7.))*
	                     .75 * (.5 + .5 * cos (p.z * 7.));
		float d = length (p) - r;
	    return d;
	}

	float map (in vec3 p) {
	    vec3 pBottom = p;
	    vec3 pTop = p;

	    float r1 = .1 + .3 * (.5 + .5 * sin (2. * iTime));
	    float r2 = .15 + .2 * (.5 + .5 * sin (3. * iTime));
	    float r3 = .2 + .2 * (.5 + .5 * sin (4. * iTime));
	    float r4 = .25 + .1 * (.5 + .5 * sin (5. * iTime));

	    float t = 2. * iTime;
	    vec3 offset1 = vec3 (-.1*cos(t), .1, -.2*sin(t));
	    vec3 offset2 = vec3 (.2, .2*cos(t), .3*sin(t));
	    vec3 offset3 = vec3 (-.2*cos(t), -.2*sin(t), .3);
	    vec3 offset4 = vec3 (.1, -.4*cos(t), .4*sin(t));
	    vec3 offset5 = vec3 (.4*cos(t), -.2, .3*sin(t));
	    vec3 offset6 = vec3 (-.2*cos(t), -.4, -.4*sin(t));
	    vec3 offset7 = vec3 (.3*sin(t), -.6*cos(t), .6);
	    vec3 offset8 = vec3 (-.3, .5*sin(t), -.4*cos(t));

	    float ball1 = sdSphere (p + offset1, r4);
	    float ball2 = sdSphere (p + offset2, r2);
		float metaBalls = opCombine (ball1, ball2, r1);

	    ball1 = sdSphere (p + offset3, r1);
	    ball2 = sdSphere (p + offset4, r3);
		metaBalls = opCombine (metaBalls, opCombine (ball1, ball2, .2), r2);

	    ball1 = sdSphere (p + offset5, r3);
	    ball2 = sdSphere (p + offset6, r2);
		metaBalls = opCombine (metaBalls, opCombine (ball1, ball2, .2), r3);

	    ball1 = sdSphere (p + offset7, r3);
	    ball2 = sdSphere (p + offset8, r4);
		metaBalls = opCombine (metaBalls, opCombine (ball1, ball2, .2), r4);

	    pBottom.yz *= r2d(90.);
	    vec2 cellBottom = opRepeat2 (pBottom.yx, vec2 (.75));

	    pTop.yz *= r2d(270.);
	    vec2 cellTop = opRepeat2 (pTop.yx, vec2 (.75));

	    float hexBottom = sdHexPrism (pBottom + vec3 (.0, .0, -3.), vec2 (.25, .75 + .2 * sin(cellBottom.y)*cos(cellBottom.x)));
	    float hexTop = sdHexPrism (pTop + vec3 (.0, .0, -3.), vec2 (.25, .75 + .2 * sin(cellTop.y)*cos(cellTop.x)));

	    //float thing = sdThing (p-vec3 (.0, .0, 6.));

	    return min (metaBalls, min (hexBottom, hexTop));
	}

	float march (in vec3 ro, in vec3 rd, out int iter) {
	    float t = .0;
	    float d = .0;
	    iter = 0;
	    for (int i = 0; i < 64; ++i) {
	        iter++;
	        vec3 p = ro + d * rd;
	        t = map (p);
	        if (t < .0001) break;
	        d += t*.975;
	    }

	    return d;
	}

	vec3 normal (in vec3 p) {
		float d = map (p);
	    vec3 e = vec3 (.001, .0, .0);
	    return normalize (vec3 (map (p + e.xyy) - d,
	                            map (p + e.yxy) - d,
	                            map (p + e.yyx) - d));
	}

	vec3 shade (in vec3 ro, in vec3 p) {
	    vec3 amb = vec3 (.01);
		vec3 diffC = vec3 (1., .0, .0);
	    vec3 specC = vec3 (.0);

	    vec3 n = normal (p);
	    vec3 lPos = ro + vec3 (.5, 1.0, -3.);
	    vec3 lDir = lPos - p;
	    vec3 lnDir = normalize (lDir);
	    float sha = 1.;

	    float diff = max (dot (n, lnDir), .0);
	    float spec = .0;
	    
		return amb + sha * (diff * diffC + spec * specC);
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

	void main () {
		vec2 uv = fragCoord.xy;
	    uv = uv *2. - 1.;
	    uv.x *= iResolution.x / iResolution.y;

	    // set up "camera", view origin (ro) and view direction (rd)
	    float angle = radians (300. + 55. * iTime);
	    float dist = 3. + cos (1.5*iTime);
	    vec3 ro = vec3 (dist * cos (angle), .0, dist * sin (angle));
	    vec3 aim = vec3 (.0);
	    float zoom = 2.;
	    vec3 rd = camera (uv, ro, aim, zoom);

	    int iter = 0;
	    float d = march (ro, rd, iter);
	    vec3 p = ro + d * rd;
	    
	    vec3 n = normal (p);
	    vec3 col = shade (ro, p);
	    col = mix (col, vec3 (.95, .85, .7), pow (1. - 1. / d, 10.));

	    //col = col / (1. + col);
	    col = sqrt (col);

		fragColor = vec4 (col, 1.);
	}
);

const char fragPathTracer[] = GLSL(
    // the uniform inputs taken over from shadertoy.com
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

	const int MAX_SAMPLES = 12;
	const int MAX_BOUNCES = 4;

	struct Ray {
		vec3 ro; // ray origin
	    vec3 rd; // ray direction
	};

	struct Result {
	    bool  hit;    // did a ray hit anything?
	    vec3  point;  // where did it hit in world-space
	    vec3  normal; // surface-normal at the hit-point
	    float dist;   // distance from ro to hit-point
	    int   id;     // material-id at that hit-point
	};

	struct Material {
	    vec3  diffuse;     // base-color
	    float emissive;    // light/emissive strength
	    bool  doesReflect; // flag indicating if surface reflects
	    float reflAmount;  // factor for reflection influence
	    bool  doesRefract; // flag indicating if surface refracts
	    float ior;         // index of refraction
	};

	const Material material[6] = Material[6] (Material (vec3 (.5, .7, 1.), // Blinn/Phong: diffuse
	                                              .0,                // Blinn/Phong: emissive
	                                              true,             // does reflect
	                                              .95,               // reflection strength
	                                              false,             // does refract
	                                              .31),              // ior

	                                    Material (vec3 (.2, 1., .2), // Blinn/Phong: diffuse
	                                              .0,                // Blinn/Phong: emissive
	                                              true,              // does reflect
	                                              .85,               // reflection strength
	                                              false,             // does refract
	                                              .31),              // ior

	                                    Material (vec3 (1., .2, .2), // Blinn/Phong: diffuse
	                                              .0,                // Blinn/Phong: emissive
	                                              true,              // does reflect
	                                              .75,               // reflection strength
	                                              false,             // does refract
	                                              .31),              // ior

	                                    Material (vec3 (.25, .5, 1.),  // Blinn/Phong: diffuse
	                                              12.,                 // Blinn/Phong: emissive
	                                              false,               // does reflect
	                                              .15,                 // reflection strength
	                                              false,               // does refract
	                                              .85),                // ior

	                                    Material (vec3 (.85),          // Blinn/Phong: diffuse
	                                              .0,                  // Blinn/Phong: emissive
	                                              false,               // does reflect
	                                              .15,                 // reflection strength
	                                              false,               // does refract
	                                              .85),                // ior
	                                 
	                                    Material (vec3 (1., .75, .5),  // Blinn/Phong: diffuse
	                                              12.,                  // Blinn/Phong: emissive
	                                              false,               // does reflect
	                                              .15,                 // reflection strength
	                                              false,               // does refract
	                                              .85));               // ior

	const float PI = 3.14159265359;
	const Result nullResult = Result (false, vec3 (.0), vec3 (.0), .0, 0);
	mat2 r2d (in float r) { float c = cos (r); float s = sin (r); return mat2 (vec2 (c, s), vec2 (-s, c));}
	Result minResult (in Result a, in Result b) {
	    if (!a.hit)
	        return b;

	    if (!b.hit)
	        return a;

	    bool favourA = a.dist <= b.dist;
	    return  Result (true,
	                    favourA ? a.point : b.point,
	                    favourA ? a.normal : b.normal,
	                    favourA ? a.dist : b.dist,
	                    favourA ? a.id : b.id);
	}

	Result sphereIntersect (in Ray ray, in vec3 p, in float r, in int id) {
	    Result res = nullResult;

	    // set up coefficients a, b and c
	    float b = dot (2. * ray.rd, ray.ro - p);
	    vec3 op = ray.ro - p;
	    float c = dot (op, op) - r * r;
	    float discriminant = b * b - 4. * c;
	    if (discriminant < .0) return res;
	    float d = sqrt (discriminant);

	    // compute possible values for t
	    float t1 = (-b + d) * .5;
	    float t2 = (-b - d) * .5;

	    if (t1 > .0 && t2 > .0) {
	        if (t1 < t2) {
	            vec3 i1 = ray.ro + t1 * ray.rd;
	            float d1 = distance (i1, ray.ro);
	            res.hit = true;
	            res.point = i1;
	            res.normal = normalize (i1 - p);
	            res.dist = d1;
	            res.id = id;
	        } else {
	            vec3 i2 = ray.ro + t2 * ray.rd;
	            float d2 = distance (i2, ray.ro);
	            res.hit = true;
	            res.point = i2;
	            res.normal = normalize (i2 - p);
	            res.dist = d2;
	            res.id = id;
	        }
	    }

	    return res;
	}

	Result planeIntersect (in Ray ray, in vec3 p, in vec3 n, in int id) {
	    Result res = nullResult;

		// are ray and plane parallel?
	    if (dot (n, ray.rd) > 1e-6) {
	        return res;
	    }

	    // determine ray-plane intersection point
	    vec3 i = ray.ro + (dot(p - ray.ro, n) / dot (ray.rd, n)) * ray.rd;

	    // prepare result
	    res.hit = true;
	    res.point = i;
	    res.normal = normalize (n);
	    res.dist = distance (i, ray.ro);
	    res.id = id;

	    return res;
	}

	Ray cameraRay (in vec2 uv, in vec3 ro, in vec3 lookAt, in float zoom) {
		Ray ray = Ray (vec3 (.0), vec3 (.0));

	    vec3 worldUp = vec3 (.0, 1., .0);
	    vec3 forward = normalize (lookAt - ro);
	    vec3 right = normalize (cross (worldUp, forward));
	    vec3 up = normalize (cross (forward, right));
	    vec3 camCenter = ro + zoom * forward;
	    vec3 i = camCenter + uv.x * right + uv.y * up;
	    vec3 rd = normalize (i - ro);

	    ray.ro = ro;
	    ray.rd = rd;

		return ray;
	}

	vec3 lightPos() {
		return vec3 (-1.75 + cos (iTime), 1.25, .0+ sin(iTime));
	}

	float lightSize () {
		return .4;
	}

	Result lightIntersect (in Ray ray) {
		return sphereIntersect (ray, lightPos(), lightSize(), 5);
	}

	bool intersectShadow (in Ray ray, in float dist) {
	    Result ball = sphereIntersect (ray, vec3 (1.5, -1., 2.0), 1., 4);
	    if (ball.dist > .0001 && ball.dist < dist)
	        return true;

	    return false;
	}

	Result intersect (in Ray ray) {
	    Result res = nullResult;

	    Result ball = sphereIntersect (ray, vec3 (1.5, -1., 2.0), 1., 0);
	    Result light = lightIntersect (ray);

	    Result plane1 = planeIntersect (ray, vec3 (.0, -2., .0), vec3 (.0, 1., .0), 4);
	    Result plane2 = planeIntersect (ray, vec3 (.0, .0, 4.), vec3 (.0, .0, -1.), 1);
	    Result plane3 = planeIntersect (ray, vec3 (-4.0, .0, 0.), vec3 (1., .0, 0.), 4);
	    Result plane4 = planeIntersect (ray, vec3 (4.0, .0, 0.), vec3 (-1., .0, 0.), 0);
	    Result plane5 = planeIntersect (ray, vec3 (.0, 2.0, 0.), vec3 (.0, -1.0, 0.), 4);
	    Result plane6 = planeIntersect (ray, vec3 (.0, .0, -4.), vec3 (.0, .0, 1.), 2);

	    res = minResult (plane1, plane2);
	    res = minResult (plane3, res);
	    res = minResult (plane4, res);
	    res = minResult (plane5, res);
	    res = minResult (plane6, res);

	    res = minResult (ball, res);
	    res = minResult (light, res);

	    return res;
	}

	float seed = .0;
	float rand() { return fract (sin (seed++) * 43758.5453123); }

	vec3 cosineDirection (in vec3 n) {
		float u = rand();
	    float v = rand();

	    float a = 6.2831853 * v;
	    u = 2. * u - 1.;
	    return normalize( n + vec3(sqrt(1.-u*u) * vec2(cos(a), sin(a)), u) );
	}

	vec3 randomSphereDirection () {
	    vec2 r = vec2 (rand () * 6.2831, rand() * 6.2831);
		vec3 dr = vec3 (sin (r.x) * vec2 (sin (r.y), cos (r.y)), cos (r.x));
		return dr;
	}

	vec3 sampleLight (in vec3 ro) {
	    vec3 n = randomSphereDirection() * lightSize();
	    return lightPos() + n;
	}

	vec3 bsdf (in vec3 rd, in Result res) {
	    // specular - brdf - not correct yet
	    if (material[res.id].doesReflect) {
	        vec3 ref = normalize (reflect (rd, res.normal));
	        if (material[res.id].reflAmount < rand ()) {
				return ref;
	        } else {
				return cosineDirection (ref);
	        }
	    }

	    // transparent - btdf - not implemented
	    if (material[res.id].doesRefract) {
			return cosineDirection (res.normal);
	    }

	    // diffuse - brdf
		return cosineDirection (res.normal);
	}

	vec3 trace (in Ray ray, in int bounces)
	{
	    vec3 tcol = vec3 (.0);
	    vec3 fcol = vec3 (1.);
	    bool specBounce = true;

	    // create light-paths with bounces segments/depth iteratively
	    for (int i = 0; i < bounces; ++i)
	    {
	        // intersect scene
	        Result res = intersect (ray);

	        // hit light-source
	        if (material[res.id].emissive > .0) {
	            if (specBounce)
		            tcol += fcol * material[res.id].diffuse * material[res.id].emissive;

	            return tcol;
	        }

	        // prepare ray for indirect lighting gathering
	        ray.rd = bsdf (ray.rd, res);
	        ray.ro = res.point;
	        specBounce = false;

	        fcol *= material[res.id].diffuse;

	        // next event estimation
	        vec3 ld = sampleLight (ray.ro) - ray.ro;
	        vec3 nld = normalize (ld);
	        float lSize = lightSize ();
	        vec3 lDir = lightPos () - ray.ro;
	        if (!specBounce && i < bounces - 1 && !intersectShadow (Ray (ray.ro, nld), length (ld))) {
	            float cos_a_max = sqrt (1. - clamp (lSize * lSize / dot (lDir, lDir), 0., 1.));
	            float weight = 2. * (1. - cos_a_max);
	            vec3 lightColor = material[5].emissive * material[5].diffuse;
	            tcol += (fcol * lightColor) * (weight * clamp (dot (nld, res.normal), 0., 1.));
	        }
	    }

	    return tcol;
	}

	vec3 getColor (in Ray ray)
	{
	    vec3 col = vec3 (.0);

	    for (int i = 0; i < MAX_SAMPLES; ++i) {
	        // accumulate path
	        col += trace (ray, MAX_BOUNCES);
	    }
	    col = col / float (MAX_SAMPLES);

	    // apply tonemapping & gamma correction
	    col = col / (1. + col);
	    col = sqrt (col);

	    return col;
	}

	void main () {
	    // normalize and aspect-correct UVs
	    vec2 uv = fragCoord.xy;
	    uv = uv * 2. - 1.;
	    float aspect = iResolution.x / iResolution.y;
	    uv.x *= aspect;

	    // generate primary ray from camera
	    vec3 ro = vec3 (.0, .0, -3.);
	    ro.xz *= r2d (PI * sin (.5*PI*(iMouse.x/iResolution.x * 2. - 1.)));
	    ro.yz *= r2d (.25 * PI * sin (-.5*PI*(iMouse.y/iResolution.y * 2. - 1.)));
	    vec3 lookAt = vec3 (.0);
	    float zoom = 1.75;
	    Ray ray = cameraRay (uv, ro, lookAt, zoom);

	    // init seed for the random hemisphere sampling
	    vec2 tmp = uv + vec2 (1. + fract (iTime));
		seed = dot (tmp, tmp);

	    // pull pixel-value from previous frame and average it with current value
	    vec3 col = getColor (ray);

		fragColor = vec4 (col, 1.);
	}
);

const char fragRaytracer[] = GLSL(
    // the uniform inputs taken over from shadertoy.com
    uniform vec3 iResolution;
    uniform float iTime;
    uniform float iTimeDelta;
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

	struct Ray {
		vec3 ro; // ray origin
	    vec3 rd; // ray direction
	};

	struct Result {
	    bool  hit;    // did a ray hit anything?
	    vec3  point;  // where did it hit in world-space
	    vec3  normal; // surface-normal at the hit-point
	    float dist;   // distance from ro to hit-point
	    int   id;     // material-id at that hit-point
	};

	struct Light {
	    vec3 position;
		vec3 ambient;
	    vec3 diffuse;
	    vec3 specular;
	    float attenuation;
	};

	struct Material {
	    // PBR part
	    vec3  albedo;    // base-color
	    float metallic;  // .0 dielectric, 1. metal
	    float roughness; // .0 .. 1. polished to rough/dull

	    // Blinn/Phong part
	    vec3  ambient;     // fake GI-term
	    vec3  diffuse;     // base-color
	    vec3  specular;    // color of specular highlight under white light
	    float shininess;   // "hardness" of surface
	    vec3  emissive;    // light emitted?
	    bool  doesReflect; // flag indicating if surface reflects
	    float reflAmount;  // factor for reflection influence
	    bool  doesRefract; // flag indicating if surface refracts
	    float ior;         // index of refraction
	};

	float saturate (in float v) {return clamp (v, .0, 1.);}

	const Result nullResult = Result (false, vec3 (.0), vec3 (.0), .0, 0);

	Result minResult (in Result a, in Result b) {
	    if (!a.hit)
	        return b;

	    if (!b.hit)
	        return a;

	    bool favourA = a.dist <= b.dist;
	    return  Result (true,
	                    favourA ? a.point : b.point,
	                    favourA ? a.normal : b.normal,
	                    favourA ? a.dist : b.dist,
	                    favourA ? a.id : b.id);
	}

	Result sphereIntersect (in Ray ray, in vec3 p, in float r, in int id) {
	    Result res = nullResult;

	    float a = dot (ray.rd, ray.rd);

	    // exit early, if... ? 
	    if (a <= 1e-6) {
	        return res;
	    }

	    // set up coefficients a, b and c
	    float b = dot (2. * ray.rd, ray.ro - p);
	    vec3 op = ray.ro - p;
	    float c = dot (op, op) - r * r;
	    float d = sqrt (b * b - 4. * a * c);
	    float twoA = 1. / 2.*a;

	    // compute possible values for t
	    float t1 = (-b + d) * twoA;
	    float t2 = (-b - d) * twoA;

	    // this case should not be possible 
	    if (t1 <= .0 && t2 <= .0) {
			return res;
	    }

	    if (t1 > .0 && t2 > .0) {
	        if (t1 < t2) {
	            vec3 i1 = ray.ro + t1 * ray.rd;
	            float d1 = distance (i1, ray.ro);
	            res.hit = true;
	            res.point = i1;
	            res.normal = normalize (i1 - p);
	            res.dist = d1;
	            res.id = id;
	        } else {
	            vec3 i2 = ray.ro + t2 * ray.rd;
	            float d2 = distance (i2, ray.ro);
	            res.hit = true;
	            res.point = i2;
	            res.normal = normalize (i2 - p);
	            res.dist = d2;
	            res.id = id;
	        }
	    }

	    return res;
	}

	Result planeIntersect (in Ray ray, in vec3 p, in vec3 n, in int id) {
	    Result res = nullResult;

		// are ray and plane parallel?
	    if (dot (n, ray.rd) > 1e-6) {
	        return res;
	    }

	    // determine ray-plane intersection point
	    vec3 i = ray.ro + (dot(p - ray.ro, n) / dot (ray.rd, n)) * ray.rd;


	    // prepare result
	    res.hit = true;
	    res.point = i;
	    res.normal = normalize (n);
	    res.dist = distance (i, ray.ro);
	    res.id = id;

	    return res;
	}

	Result cylinderIntersect(in Ray ray, in vec3 p, in vec3 n, in float h , in float r, in int id){
	    Result res = nullResult;
		return res;
	}

	Ray cameraRay (in vec2 uv, in vec3 ro, in vec3 lookAt, in float zoom) {
		Ray ray = Ray (vec3 (.0), vec3 (.0));

	    vec3 worldUp = vec3 (.0, 1., .0);
	    vec3 forward = normalize (lookAt - ro);
	    vec3 right = normalize (cross (worldUp, forward));
	    vec3 up = normalize (cross (forward, right));
	    vec3 camCenter = ro + zoom * forward;
	    vec3 i = camCenter + uv.x * right + uv.y * up;
	    vec3 rd = normalize (i - ro);

	    ray.ro = ro;
	    ray.rd = rd;

		return ray;
	}

	Result trace (in Ray ray) {
	    Result ball1 = sphereIntersect (ray,
	                                    vec3 (-2., -1.4*(cos(iTime)*.5+.5), -.5), // center of sphere
	                                    .6,                  // radius of sphere
	                                    3);                   // material-id
	    Result ball2 = sphereIntersect (ray,
	                                    vec3 (1., -1.5*(cos(1.+iTime)*.5+.5), .5),  // center of sphere
	                                    .5,                  // radius of sphere
	                                    3);                   // material-id
	    Result ball3 = sphereIntersect (ray,
	                                    vec3 (.0, -1.6*(cos(2.+iTime)*.5+.5), 1.5),  // center of sphere
	                                    .4,                   // radius of sphere
	                                    3);                   // material-id

	    Result plane1 = planeIntersect (ray,
	                                    vec3 (.0, -2., .0),   // point on plane
	                                    vec3 (.0, 1., .0),    // normal of plane
	                                    1);                   // material-id
	    Result plane2 = planeIntersect (ray,
	                                    vec3 (.0, .0, 4.),    // point on plane
	                                    vec3 (.0, .0, -1.),   // normal of plane
	                                    2);                   // material-id
	    Result plane3 = planeIntersect (ray,
	                                    vec3 (-6.0, .0, 0.),  // point on plane
	                                    vec3 (1., .0, 0.),    // normal of plane
	                                    0);                   // material-id
	    Result plane4 = planeIntersect (ray,
	                                    vec3 (6.0, .0, 0.),   // point on plane
	                                    vec3 (-1., .0, 0.),   // normal of plane
	                                    0);                   // material-id
	    Result plane5 = planeIntersect (ray,
	                                    vec3 (.0, 2.0, 0.),   // point on plane
	                                    vec3 (.0, -1.0, 0.),  // normal of plane
	                                    1);                   // material-id
	    Result plane6 = planeIntersect (ray,
	                                    vec3 (.0, .0, -4.),   // point on plane
	                                    vec3 (.0, .0, 1.),    // normal of plane
	                                    2);                   // material-id

	    Result res = minResult (plane1, plane2);
		res = minResult (plane3, res);
		res = minResult (plane4, res);
		res = minResult (plane5, res);
		res = minResult (plane6, res);

	    res = minResult (ball1, res);
	    res = minResult (ball2, res);
		res = minResult (ball3, res);

	    return res;
	}

	Light light[2] = Light[2] (Light (vec3 (1., .5, -1.), // position
	                                  vec3 (.1),          // ambient
	                                  vec3 (1., 1., .5),  // diffuse
	                                  vec3 (1.),          // specular
	                                  1.),                // attenuation
	                           
	                           Light (vec3 (-2., 1., 2.), // position
	                                  vec3 (.1),          // ambient
	                                  vec3 (.5, 1., 1.),  // diffuse
	                                  vec3 (1.),          // specular
	                                  1.));               // attenuation

	Material material[4] = Material[4] (Material (vec3 (.5),         // PBR: albedo
	                                              .0,                // PBR: metallic
	                                              1.,                // PBR: roughness
	                                              vec3 (.1),         // Blinn/Phong: ambient
	                                              vec3 (.3, .6, .9), // Blinn/Phong: diffuse
	                                              vec3 (1.),         // Blinn/Phong: specular
	                                              20.,               // Blinn/Phong: shininess
	                                              vec3 (.0),         // Blinn/Phong: emissive
	                                    		  false,             // does reflect
	                                              .05,                // reflection strength
	                                    		  false,             // does refract
	                                    		  .31),             // ior

	                                    Material (vec3 (.5),         // PBR: albedo
	                                              .0,                // PBR: metallic
	                                              1.,                // PBR: roughness
	                                              vec3 (.1),         // Blinn/Phong: ambient
	                                              vec3 (.9, .3, .6), // Blinn/Phong: diffuse
	                                              vec3 (1.),         // Blinn/Phong: specular
	                                              20.,               // Blinn/Phong: shininess
	                                              vec3 (.0),         // Blinn/Phong: emissive
	                                    		  true,             // does reflect
	                                              .05,                // reflection strength
	                                    		  false,             // does refract
	                                    		  .31),             // ior

	                                    Material (vec3 (.5),         // PBR: albedo
	                                              .0,                // PBR: metallic
	                                              1.,                // PBR: roughness
	                                              vec3 (.1),         // Blinn/Phong: ambient
	                                              vec3 (.6, .9, .3), // Blinn/Phong: diffuse
	                                              vec3 (1.),         // Blinn/Phong: specular
	                                              20.,               // Blinn/Phong: shininess
	                                              vec3 (.0),         // Blinn/Phong: emissive
	                                    		  true,              // does reflect
	                                              .05,                // reflection strength
	                                    		  false,             // does refract
	                                    		  .31),             // ior

	                                    Material (vec3 (.5),         // PBR: albedo
	                                              .0,                // PBR: metallic
	                                              1.,                // PBR: roughness
	                                              vec3 (.1),         // Blinn/Phong: ambient
	                                              vec3 (.95, .9, .85), // Blinn/Phong: diffuse
	                                              vec3 (1.),         // Blinn/Phong: specular
	                                              20.,               // Blinn/Phong: shininess
	                                              vec3 (.0),         // Blinn/Phong: emissive);
	                                    		  true,              // does reflect
	                                              .25,               // reflection strength
	                                    		  true,             // does refract
	                                    		  .31));            // ior

	vec3 shade (in Ray ray, in Result res) {
	    vec3 amb = vec3 (.1);
	    vec3 diffC[4];

		float pattern1 = saturate (pow (abs(15. * cos(res.point.x+iTime) * sin (res.point.z+iTime)), .3));
	    float pattern2 = saturate (pow (abs(2. * cos(res.point.x+iTime) * sin (res.point.z+iTime) * .5 + .5), .3));
	    float pattern3 = saturate (pow (length (4.*sin(mod((res.point.y*res.point.x), .3))), .125));
	    float pattern4 = saturate (mod (length(res.point*sin(.1*iTime)), .5));

	    diffC[0] = mix (vec3 (.3, .6, .9), vec3 (.8), 1. - pattern1);
	    diffC[1] = mix (vec3 (.9, .3, .6), vec3 (.9), 1. - pattern2);
	    diffC[2] = mix (vec3 (.6, .9, .3), vec3 (.5), 1. - pattern4);
	    diffC[3] = vec3 (.95, .9, .85);
	    vec3 specC = vec3 (1.);
	    vec3 specC2 = vec3 (1.);
	    float shininess = 20.;
	    float shininess2 = 20.;

	    vec3 lCol = vec3 (.95, .95, .75);
	    vec3 lPos = vec3 (cos (2. * -iTime) * 3., .5, 1. + sin (1.5 *iTime) * .75);
	    vec3 lDir = normalize (lPos - res.point);
	    float diff = max (dot (res.normal, lDir), .0);
	    vec3 ref = normalize (reflect (ray.rd, res.normal));
	    float spec = pow (clamp (dot (ref, lDir), .0, 1.), shininess);
	    Result shaRes = trace (Ray (res.point, lDir));
	    float lDist = length (lPos - res.point);
	    float attenuation = 2. / (lDist * lDist);
	    lCol *= attenuation;
	    specC *= attenuation;
	    float sha = shaRes.dist < lDist ? .5 : 1.;

	    vec3 lCol2 = vec3 (.75, .95, .95);
	    vec3 lPos2 = vec3 (sin (iTime) * 4., -.25, 1. - cos (iTime) * .5);
	    vec3 lDir2 = normalize (lPos2 - res.point);
	    float diff2 = max (dot (res.normal, lDir2), .0);
	    vec3 ref2 = normalize (reflect (ray.rd, res.normal));
	    float spec2 = pow (clamp (dot (ref2, lDir2), .0, 1.), shininess2);
	    Result shaRes2 = trace (Ray (res.point, lDir2));
	    float lDist2 = length (lPos2 - res.point);
	    float attenuation2 = 3. / (lDist2 * lDist2);
	    lCol2 *= attenuation2;
	    specC2 *= attenuation2;
	    float sha2 = shaRes2.dist < lDist2 ? .5 : 1.;

	    vec3 col = amb +
	        	   sha * (diff * diffC[res.id] * lCol) +
				   (sha >= .5 ? .0 : spec) * specC +
				   sha2 * (diff2 * diffC[res.id] * lCol2) +
	               (sha2 >= .5 ? .0 : spec2) * specC;

	    return col;
	}

	void main () {
	    // normalize and aspect-correct UVs
	    vec2 uv = fragCoord.xy;
	    uv = uv * 2. - 1.;
	    float aspect = iResolution.x / iResolution.y;
	    uv.x *= aspect;

	    // generate primary ray from camera
	    float r = 3.;
	    float theta = radians (180. - sin((iMouse.y/iResolution.y * 2.) - 1.)*35.);
	    float rho = radians (sin((iMouse.x/iResolution.x * 2.) - 1.)*120.);
	    float x = r * cos (theta) * sin (rho);
	    float y = r * sin (theta) * sin (rho);
	    float z = r * cos (theta);
	    vec3 ro = vec3 (x, y, z);
	    vec3 lookAt = vec3 (.0);
	    float zoom = 1.75;
	    Ray ray = cameraRay (uv, ro, lookAt, zoom);

	    // ray-trace the scene (primary view-ray)
	    Result res = trace (ray);
	    vec3 col = shade (ray, res);

	    if (material[res.id].doesReflect) {
	        // first reflection bounce
		    Ray reflectedRay = Ray (res.point, normalize (reflect (ray.rd, res.normal)));
	        Result bounce = trace (reflectedRay);
	        col += material[res.id].reflAmount * shade (reflectedRay, bounce);

	        // second reflection bounce
	        reflectedRay = Ray (bounce.point, normalize (reflect (reflectedRay.rd, bounce.normal)));
	        bounce = trace (reflectedRay);
	        col += .5 * material[res.id].reflAmount * shade (reflectedRay, bounce);
	    }

	    // tone-map and gamma-correct
	    col = col / (1. + col);
	    col = sqrt (col);

		fragColor = vec4 (col, 1.);
	}
);

const char fragWireCube[] = GLSL(
    // the uniform inputs taken over from shadertoy.com
    uniform vec3 iResolution;
    uniform float iTime;
    uniform float iTimeDelta;
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

	const int MAX_ITER    = 48;
	const float STEP_SIZE = .95;
	const float EPSILON   = .0001;
	const float PI = 3.14159265359;
	const bool DO_SHADOWS = true;
	const bool DO_REFLECTIONS = true;

	struct Result {
	    float d;
	    int id;
	};

	// PBR toolbox
	float DistributionGGX (in vec3 N, in vec3 H, in float roughness)
	{
	    float a2     = roughness * roughness;
	    float NdotH  = max (dot (N, H), .0);
	    float NdotH2 = NdotH * NdotH;

	    float nom    = a2;
	    float denom  = (NdotH2 * (a2 - 1.) + 1.);
	    denom        = PI * denom * denom;

	    return nom / denom;
	}

	float GeometrySchlickGGX (in float NdotV, in float roughness)
	{
	    float nom   = NdotV;
	    float denom = NdotV * (1. - roughness) + roughness;

	    return nom / denom;
	}

	float GeometrySmith (in vec3 N, in vec3 V, in vec3 L, in float roughness)
	{
	    float NdotV = max (dot (N, V), .0);
	    float NdotL = max (dot (N, L), .0);
	    float ggx1 = GeometrySchlickGGX (NdotV, roughness);
	    float ggx2 = GeometrySchlickGGX (NdotL, roughness);

	    return ggx1 * ggx2;
	}

	vec3 fresnelSchlick (in float cosTheta, in vec3 F0, float roughness)
	{
		return F0 + (max (F0, vec3(1. - roughness)) - F0) * pow (1. - cosTheta, 5.);
	}

	float saturate (float v)
	{
	    return clamp (v, .0, 1.);
	}

	mat2 r2d (in float a)
	{
	    float r = radians (a);
	    float c = cos (r);
	    float s = sin (r);
	    return mat2 (vec2 (c, s), vec2 (-s, c));
	}

	float sdTorus (in vec3 p, in vec2 t)
	{
	    vec2 q = vec2 (length (p.xz) - t.x, p.y);
	    return length (q) - t.y;
	}

	float sdPlane (in vec3 p, in float h)
	{
	    return length (vec3 (.0, p.y + h, .0));
	}

	float sdCylinder (in vec3 p, float h, float r)
	{
	    float d = length (p.xz) - r;
	    d = max (d, abs(p.y) - h);
	    return d;
	}

	Result scene (in vec3 p)
	{
		Result res = Result(.0, 0);

	    float floor = sdPlane (p, .7);

	    float r = .025 + .01 * cos (iTime);
	    float a = 10. * r;
	    float b = 2. * r;

	    // for construction/debugging
	    //p.xz *= r2d (iMouse.x);
	    //p.yz *= r2d (iMouse.y);

	    // for presenting
	    p.xy *= r2d (55. + 12. * iTime);
	    p.yz *= r2d (30. + 17. * iTime);
	    p.zx *= r2d (65. + 23. * iTime);
	    
	    p += vec3 (a);

	    float torus = sdTorus (p + vec3 (.0, b, .0), vec2 (b, r));
	    float bars = sdCylinder (p.yxz + vec3 (b, -a, b), a, r);
	    bars = min (bars, sdCylinder (p.yzx + vec3 (b, -a, b), a, r));
	    torus = min (bars, torus);
	    torus = min (torus, sdTorus (p.zyx + vec3 (.0, b, -2.*a), vec2 (b, r)));
	    torus = min (torus, sdCylinder (p.xzy + vec3 (-2.*a-b, -a+r, b), a-r, r));
	    torus = min (torus, sdTorus (p.xyz + vec3 (.0, b, -2.*a), vec2 (b, r)));
	    torus = min (torus, sdCylinder (p.yxz + vec3 (2.*r, -a+r, -2.*a-b), a-r, r));

	    torus = min (torus, sdTorus (p.xzy + vec3 (-2.*a+b, -2.*a-b, .0), vec2 (b, r)));
	    torus = min (torus, sdCylinder (p.xyz + vec3 (-2.*a, -a-b+2.*r, -2.*a-b), a, r));
	    torus = min (torus, sdTorus (p.xzy + vec3 (-2.*a+b, -2.*a-b, -2.*a), vec2 (b, r)));
	    torus = min (torus, sdCylinder (p.yxz + vec3 (-2.*a-b, -a+b, -2.*a-b), a, r));
	    torus = min (torus, sdTorus (p.xzy + vec3 (b, -2.*a-b, -2.*a), vec2 (b, r)));
	    torus = min (torus, sdCylinder (p.xyz + vec3 (2.*b, -a-r, -2.*a-b), a-r, r));

	    torus = min (torus, sdTorus (p.zxy + vec3 (-2.*a+b, -2.*a-b, .0), vec2 (b, r)));
	    torus = min (torus, sdCylinder (p.zyx + vec3 (-2.*a, -a, -2.*a-b), a, r));
	    torus = min (torus, sdTorus (p.zxy + vec3 (-2.*a+b, -2.*a-b, -2.*a), vec2 (b, r)));
	    torus = min (torus, sdCylinder (p.yzx + vec3 (-2.*a-b, -a+b, -2.*a-b), a, r));
	    torus = min (torus, sdTorus (p.zxy + vec3 (b, -2.*a-b, -2.*a), vec2 (b, r)));
	    torus = min (torus, sdCylinder (p.zyx + vec3 (2.*b, -a-b+r, -2.*a-b), a-r, r));

	    torus = min (torus, sdTorus (p.yxz + vec3 (-b, 2.*b, -2.*a), vec2 (b, r)));
	    torus = min (torus, sdCylinder (p.yzx + vec3 (.0, -a, 2.*b), a, r));
	    torus = min (torus, sdTorus (p.yxz + vec3 (-b, 2.*b, .0), vec2 (b, r)));
	    torus = min (torus, sdCylinder (p.zyx + vec3 (b, -a-b, 2.*b), a, r));
	    torus = min (torus, sdTorus (p.yxz + vec3 (-2.*a-b, 2.*b, .0), vec2 (b, r)));
	    torus = min (torus, sdCylinder (p.yzx + vec3 (-2.*a-2.*b, -a+r, 2.*b), a-r, r));

	    torus = min (torus, sdTorus (p.xyz + vec3 (b, -2.*a-2.*b, -2.*a+2.*r), vec2 (b, r)));
	    torus = min (torus, sdCylinder (p.zxy + vec3 (-2.*a, -a+b, -2.*a-2.*b), a, r));
	    torus = min (torus, sdTorus (p.zyx + vec3 (2.*-a+b, -2.*a-2.*b, -2.*a+b), vec2 (b, r)));
	    torus = min (torus, sdCylinder (p.xzy + vec3 (-2.*a, -a+b, -2.*a-2.*b), a, r));
	    torus = min (torus, sdTorus (p.xzy + vec3 (-2.*a, 2.*b, -b), vec2 (b, r)));
	    torus = min (torus, sdCylinder (p.zxy + vec3 (2.*b, -a, .0), a, r));

	    torus = min (torus, sdTorus (p.xzy + vec3 (.0, 2.*b, -b), vec2 (b, r)));
	    torus = min (torus, sdCylinder (p.zyx + vec3 (2.*b, -a-b, b), a, r));
	    torus = min (torus, sdTorus (p.zyx + vec3 (b, -2.*a-2.*b, -2.*a+b), vec2 (b, r)));
	    torus = min (torus, sdCylinder (p.zxy + vec3 (2.*b, -a+r, -2.*a-2.*b), a-r, r));
	    torus = min (torus, sdTorus (p.xzy + vec3 (.0, 2.*b, -2.*a-b), vec2 (b, r)));

	    res.d = min (torus, floor);
	    res.id = (res.d == torus) ? 1 : 2;

	    return res;
	}

	vec3 normal (in vec3 p)
	{
	    float d = scene (p).d;
	    vec3 e = vec3 (.001, .0, .0);
	    return normalize (vec3 (scene (p + e.xyy).d - d,
	                            scene (p + e.yxy).d - d,
	                            scene (p + e.yyx).d - d));
	}

	Result march (in vec3 ro, in vec3 rd, out int iter)
	{
	    Result res = Result (.0, 0);
	    for (int i = 0; i < MAX_ITER; ++i) {
	        iter = i;
	        vec3 p = ro + res.d * rd;
	        Result tmp = scene (p);
	        if (tmp.d < EPSILON) return res;
	        res.d += tmp.d * STEP_SIZE;
	        res.id = tmp.id;
	    }

	    return res;
	}

	float shadow (in vec3 ro, in vec3 rd)
	{
	    float result = 1.;
	    float t = .1;
	    for (int i = 0; i < MAX_ITER; i++) {
	        float h = scene (ro + t * rd).d;
	        if (h < .00001) return .0;
	        result = min (result, 8. * h/t);
	        t += h;
	    }

	    return result;
	}

	vec3 shadePBR (in vec3 ro, in vec3 rd, in float d, in int id)
	{
	    vec3 p = ro + d * rd;
	    vec3 nor = normal (p);

	    // "material" hard-coded for the moment
	    vec3 albedo =     (id == 1) ? vec3(.7,.55,.45) : (id == 2) ? vec3 (.9) : (id == 3) ? vec3 (.2, .4, .9) : vec3 (.9, .4, .1);
	    float metallic =  (id == 1) ? .9 : (id == 2) ? 0.0 : (id == 3) ? .0 : .0; 
	    float roughness = (id == 1) ? .125 : (id == 2) ? .5 : (id == 3) ? .5 : .2;
	    float ao = 1.;

	    // lights hard-coded as well atm
	    vec3 lightColors[4];
	    lightColors[0] = vec3 (.9, .9, .9) * 120.;
	    lightColors[1] = vec3 (.9, .25, .9) * 275.;
	    lightColors[2] = vec3 (.25, .9, .9) * 275.;
	    lightColors[3] = vec3 (.25, .9, .25) * 275.;

	    vec3 lightPositions[4];
	    lightPositions[0] = vec3 (.0, .0, .0);
	    lightPositions[1] = vec3 (-1.1, 1.5, -2.);
	    lightPositions[2] = vec3 (-1., .75, -2.2);
	    lightPositions[3] = vec3 (.0, 2.75, -2.2);

		vec3 N = normalize (nor);
	    vec3 V = normalize (ro - p);

	    vec3 F0 = vec3 (0.04); 
	    F0 = mix (F0, albedo, metallic);
	    vec3 kD = vec3(.0);
		           
	    // reflectance equation - cutting down on the light boost preformance of course
	    vec3 Lo = vec3 (.0);
	    for(int i = 0; i < 4; ++i) 
	    {
	        // calculate per-light radiance
	        vec3 L = normalize(lightPositions[i] - p);
	        vec3 H = normalize(V + L);
	        float distance    = length(lightPositions[i] - p);
	        float attenuation = 2.5 / (distance * distance);
	        vec3 radiance     = lightColors[i] * attenuation;        
	        
	        // cook-torrance brdf
	        float aDirect = .125 * pow (roughness + 1., 2.);
	        float aIBL = .5 * roughness * roughness;
	        float NDF = DistributionGGX(N, H, roughness);        
	        float G   = GeometrySmith(N, V, L, roughness);      
	        vec3 F    = fresnelSchlick(max(dot(H, V), 0.0), F0, roughness);       
	        
	        vec3 kS = F;
	        kD = vec3(1.) - kS;
	        kD *= 1. - metallic;	  
	        
	        vec3 nominator    = NDF * G * F;
	        float denominator = 4. * max(dot(N, V), 0.0) * max(dot(N, L), 0.0);
	        vec3 specular     = nominator / max(denominator, .001);  

	        // add to outgoing radiance Lo
	        float NdotL = max(dot(N, L), 0.0);                
	        Lo += (kD * albedo / PI + specular) * radiance * NdotL;

	        if (DO_SHADOWS) {
		    	Lo *= shadow (p, L);
	        }
	    }

	    vec3 ambient    = (kD * albedo) * ao;

	    return ambient + Lo;
	}

	float reflection (in vec3 ro, in vec3 rd, in float d)
	{
	    vec3 p = ro + d * rd;
	    vec3 nor = normal (p);
	    vec3 ref = normalize (reflect (rd, nor));
	    float iter = .001;
	    float r = 1.;
	    for (int i = 0; i < MAX_ITER; ++i) {
	    	float d = scene (p + nor *.02 + ref * iter).d;
	        if (d <=.0) break;
	        r *= saturate (d * 50.);
	        iter += max (.0, d);
	    }

	    return saturate (r);
	}

	void main ()
	{
		vec2 uv = fragCoord.xy;
	    vec2 uvRaw = uv;
	    uv = uv * 2. - 1.;
	    uv.x *= iResolution.x / iResolution.y;

	    vec3 ro = vec3 (.0, .0, -2.75);
	    vec3 rd = normalize (vec3 (uv, 1.) - ro);
	    int iter = 0;
	    Result res = march (ro, rd, iter);

	    float fog = 1. / pow(1. + res.d * res.d * .1, 3.0);
	    vec3 color = fog * shadePBR (ro, rd, res.d, res.id);

	    if (DO_REFLECTIONS) {
	    	float refl = reflection (ro, rd, res.d);
	    	color += refl * .0625;
	    }
	    color *= fog;

	    color = color / (1. + color);
	    color = .2 * color + .8 * sqrt (color);
	    color *= vec3 (.95, .9, .85);
	    color *= .2 + .8 * pow (16. * uvRaw.x * uvRaw.y * (1. - uvRaw.x) * (1. - uvRaw.y), .3);

	    fragColor = vec4 (color, 1.);
	}
);

const char fragPBRPlayground[] = GLSL(
    // the uniform inputs taken over from shadertoy.com
    uniform vec3 iResolution;
    uniform float iTime;
    uniform float iTimeDelta;
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

	const int MAX_ITER    = 64;
	const float STEP_SIZE = .95;
	const float EPSILON   = .001;
	const float PI = 3.14159265359;

	float saturate (in float v) { return clamp (v, .0, 1.); }
	mat3 rotX (in float a) {float c = cos(a); float s = sin (a); return mat3 (vec3 (1., .0, .0), vec3 (.0, c, s), vec3 (.0, -s, c));}
	mat3 rotY (in float a) {float c = cos(a); float s = sin (a); return mat3 (vec3 (c, .0, s), vec3 (.0, 1., .0), vec3 (-s, .0, c));}
	mat3 rotZ (in float a) {float c = cos(a); float s = sin (a); return mat3 (vec3 (c, s, .0), vec3 (-s, c, .0), vec3 (.0, .0, 1.));}
	mat2 r2d (in float a) { float c = cos(a); float s = sin (a); return mat2 (vec2 (c, s), vec2 (-s, c));}

	struct Result {
		float d;
		int id;
	};

	// noise toolbox
	float hash (float f)
	{
		return fract (sin (f) * 45734.5453);
	}

	float noise3d (vec3 p)
	{
	    vec3 u = floor (p);
	    vec3 v = fract (p);
	    
	    v = v * v * (3. - 2. * v);

	    float n = u.x + u.y * 57. + u.z * 113.;
	    float a = hash (n);
	    float b = hash (n + 1.);
	    float c = hash (n + 57.);
	    float d = hash (n + 58.);

	    float e = hash (n + 113.);
	    float f = hash (n + 114.);
	    float g = hash (n + 170.);
	    float h = hash (n + 171.);

	    float result = mix (mix (mix (a, b, v.x),
	                             mix (c, d, v.x),
	                             v.y),
	                        mix (mix (e, f, v.x),
	                             mix (g, h, v.x),
	                             v.y),
	                        v.z);

	    return result;
	}

	float fbm (vec3 p)
	{
		mat3 m1 = mat3 (rotZ (23.4));
		mat3 m2 = mat3 (rotZ (45.5));
		mat3 m3 = mat3 (rotZ (77.8));

	    float result = .0;
	    result = 0.5 * noise3d (p);
	    p *= m1 * 2.02;
	    result += 0.25 * noise3d (p);
	    p *= m2 * 2.03;
	    result += 0.125 * noise3d (p);
	    p *= m3 * 2.04;
	    result += 0.0625 * noise3d (p);
	    result /= 0.9375;

	    return result;
	}

	// basic sdf toolbox
	vec3 opRepeat (in vec3 p, in vec3 size) {return mod (p, 2. * size) - size;}
	float sdTorus (in vec3 p, in vec2 t) { vec2 q = vec2 (length (p.xz) - t.x, p.y); return length (q) - t.y; }
	float sdBox (in vec3 p, in vec3 size, in float r) { return length (max (abs (p) - size, .0)) - r; }
	float sdSphere (in vec3 p, float r) { return length (p) - r; }
	vec2 opRepeat2 (inout vec2 p,in vec2 s) {vec2 h=.5*s; vec2 c=floor((p+h)/s); p=mod(p+h,s)-h; return c;}

	// PBR toolbox
	float DistributionGGX (in vec3 N, in vec3 H, in float roughness)
	{
	    float a2     = roughness * roughness;
	    float NdotH  = max (dot (N, H), .0);
	    float NdotH2 = NdotH * NdotH;

	    float nom    = a2;
	    float denom  = (NdotH2 * (a2 - 1.) + 1.);
	    denom        = PI * denom * denom;

	    return nom / denom;
	}

	float GeometrySchlickGGX (in float NdotV, in float roughness)
	{
	    float nom   = NdotV;
	    float denom = NdotV * (1. - roughness) + roughness;

	    return nom / denom;
	}

	float GeometrySmith (in vec3 N, in vec3 V, in vec3 L, in float roughness)
	{
	    float NdotV = max (dot (N, V), .0);
	    float NdotL = max (dot (N, L), .0);
	    float ggx1 = GeometrySchlickGGX (NdotV, roughness);
	    float ggx2 = GeometrySchlickGGX (NdotL, roughness);

	    return ggx1 * ggx2;
	}

	vec3 fresnelSchlick (in float cosTheta, in vec3 F0, float roughness)
	{
		return F0 + (max (F0, vec3(1. - roughness)) - F0) * pow (1. - cosTheta, 5.);
	}

	// ray-marching stuff
	Result scene (in vec3 p)
	{
	    float floor = p.y + .75;

		p -= vec3 (.0, .75, 2.5);
		vec3 p2 = p;
	    float gridCount = 5.;
	    float range = .55;
	    float hRange = range * .5;
	    p = vec3 (
	        fract ((p.x + hRange) / range) * range - hRange,
	        fract ((p.y + hRange) / range) * range - hRange,
	        p.z);
	    float limitX = abs (p2.x) - gridCount * hRange;
	    float limitY = abs (p2.y) - gridCount * hRange;
	    float limits = max (limitX, limitY);
		float spheres = max (sdSphere (p, .25), limits);

	    Result res = Result (.0, 0);
		res.d = min (spheres, floor);
	    res.id = (res.d == floor ) ? 1 : 2;
	    return res;
	}

	Result trace (in vec3 ro, in vec3 rd)
	{
	    Result res = Result (.0, 0);

	    for (int i = 0; i < MAX_ITER; i++)
	    {
	        vec3 p = ro + res.d * rd;
	        Result tmp = scene (p);
	        if (tmp.d < EPSILON) return res;
	        res.d += tmp.d * STEP_SIZE;
	        res.id = tmp.id;
	    }

	    return res;
	}

	vec3 normal (in vec3 p)
	{
	    vec3 e = vec3(.0001, .0, .0);
	    float d = scene (p).d;
	    vec3 n = vec3 (scene (p + e.xyy).d - d, scene (p + e.yxy).d - d, scene (p + e.yyx).d - d);
	    return normalize(n);
	}

	float shadow (in vec3 ro, in vec3 rd)
	{
	    float result = 1.;
	    float t = .1;
	    for (int i = 0; i < MAX_ITER; i++) {
	        float h = scene (ro + t * rd).d;
	        if (h < 0.00001) return .0;
	        result = min (result, 8. * h/t);
	        t += h;
	    }

	    return result;
	}

	vec3 shadePBR (in vec3 ro, in vec3 rd, in float d, in int id)
	{
	    vec3 p = ro + d * rd;
	    vec3 nor = normal (p);

	    // "material" hard-coded for the moment
	    float mask1 = saturate (pow (fbm(4.*p), 3.15));
	    float mask2 = saturate (pow (fbm(6.*p), 4.5));
		float mask = (id == 1) ? mask1 : mask2;
	    vec3 albedo1 = mix (vec3 (1., .7, .5), vec3 (.9, .5, .3), smoothstep(.1, .09, mask));
	    vec3 albedo2 = mix (vec3 (.1, .2, .7), vec3 (.2, .8, .4), smoothstep(.1, .09, mask));
	    vec3 albedo = (id == 1) ? albedo1 : albedo2 ;
	    float metallic = (id == 1) ? 1.: .0;
	    float roughness = (id == 1) ? .75: mask;
	    float ao = 1.;

	    // lights hard-coded as well atm
	    vec3 lightColors[4];
	    lightColors[0] = vec3 (.9, .9, .9) * 15.;
	    lightColors[1] = vec3 (.9, .25, .9) * 15.;
	    lightColors[2] = vec3 (.25, .9, .9) * 15.;
	    lightColors[3] = vec3 (.25, .9, .25) * 15.;

	    vec3 lightPositions[4];
	    lightPositions[0] = p + vec3 (cos (iTime) * .5, .75 + sin (iTime)*.5, -1.5);
	    lightPositions[1] = p + vec3 (.1, 2.5, -1.);
	    lightPositions[2] = p + vec3 (-1., 1.5, -1.2);
	    lightPositions[3] = p + vec3 (1., 1.75, -1.2);

		vec3 N = normalize (nor);
	    vec3 V = normalize (ro - p);

	    vec3 F0 = vec3 (0.04); 
	    F0 = mix (F0, albedo, metallic);
	    vec3 kD = vec3(.0);
		           
	    // reflectance equation
	    vec3 Lo = vec3 (.0);
	    for(int i = 0; i < 4; ++i) 
	    {
	        // calculate per-light radiance
	        vec3 L = normalize(lightPositions[i] - p);
	        vec3 H = normalize(V + L);
	        float distance    = length(lightPositions[i] - p);
	        float attenuation = 1. / (distance * distance);
	        vec3 radiance     = lightColors[i] * attenuation;
	        
	        // cook-torrance brdf
	        float aDirect = .125 * pow (roughness + 1., 2.);
	        float aIBL = .5 * roughness * roughness;
	        float NDF = DistributionGGX(N, H, roughness);        
	        float G   = GeometrySmith(N, V, L, roughness);      
	        vec3 F    = fresnelSchlick(max(dot(H, V), 0.0), F0, roughness);       
	        
	        vec3 kS = F;
	        kD = vec3(1.) - kS;
	        kD *= 1. - metallic;	  
	        
	        vec3 nominator    = NDF * G * F;
	        float denominator = 4. * max(dot(N, V), 0.0) * max(dot(N, L), 0.0);
	        vec3 specular     = nominator / max(denominator, .001);  

	        // add to outgoing radiance Lo
	        float NdotL = max(dot(N, L), 0.0);                
	        Lo += (kD * albedo / PI + specular) * radiance * NdotL; 
		    Lo *= shadow (p, L);
	    }

	    vec3 ambient    = (kD * albedo) * ao;

	    return ambient + Lo;
	}

	float reflection (in vec3 ro, in vec3 rd, in float d)
	{
	    vec3 p = ro + d * rd;
	    vec3 nor = normal (p);
	    vec3 ref = normalize (reflect (rd, nor));
	    float iter = .001;
	    float r = 1.;
	    for (int i = 0; i < MAX_ITER; ++i) {
	    	float d = scene (p + nor *.02 + ref * iter).d;
	        if (d <=.0) break;
	        r *= saturate (d * 50.);
	        iter += max (.0, d);
	    }

	    return saturate (r);
	}

	vec3 camera (in vec2 uv, in vec3 ro, in vec3 aim, in float zoom)
	{
	    vec3 camForward = normalize (vec3 (aim - ro));
	    vec3 worldUp = vec3 (.0, 1., .0);
	    vec3 camRight = normalize (cross (camForward, worldUp));
	    vec3 camUp = normalize (cross (camRight, camForward));
	    vec3 camCenter = normalize (ro + camForward * zoom);

	    return normalize ((camCenter + uv.x*camRight + uv.y*camUp) - ro);
	}

	void main ()
	{
	    // normalizing and aspect-correction
		vec2 uvRaw = fragCoord.xy;
		vec2 uv = uvRaw;
	    uv = uv * 2. - 1.;
	    uv.x *= iResolution.x / iResolution.y;

	    // set up "camera", view origin (ro) and view direction (rd)
	    vec3 ro = vec3 (.0, .0, -.7);
	    vec3 aim = vec3 (.0, .1, .0);
	    float zoom = 1.75;
	    vec3 rd = camera (uv, ro, aim, zoom);

	    // "shake" the camera around a bit
	    rd.xy *= r2d (sin (iTime) * .15);
	    rd.xz *= r2d (cos (iTime) * .2);

	    // do the ray-march...
	    Result res = trace (ro, rd);
	    float fog = 1. / (1. + res.d * res.d * .1);
	    vec3 c = shadePBR (ro, rd, res.d, res.id);

	    // reflections (aka 2nd ray-march)... totally wrong, especially wrt to PBR
	    float refl = reflection (ro, rd, res.d);
	    c += refl * .0625;
	    c *= fog;

	    // tonemapping, "gamma-correction", tint, vignette
		c = c / (1. + c);
	    c = .2 * c + .8 * sqrt (c);
	    c *= vec3 (.9, .8, .7);
	    c *= .2 + .8 * pow (16. * uvRaw.x * uvRaw.y * (1. - uvRaw.x) * (1. - uvRaw.y), .3);

		fragColor = vec4(c, 1.);
	}
);

const char frag3DTruchet[] = GLSL(
    // the uniform inputs taken over from shadertoy.com
    uniform vec3 iResolution;
    uniform float iTime;
    uniform float iTimeDelta;
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

	const int MAX_ITER    = 128;
	const float STEP_SIZE = .95;
	const float EPSILON   = .001;
	const vec4 red     = vec4 (1.0, 0.0, 0.0, 1.0);
	const vec4 green   = vec4 (0.0, 1.0, 0.0, 1.0);
	const vec4 blue    = vec4 (0.0, 0.0, 1.0, 1.0);

	mat3 rotX (in float a) {float c = cos(a); float s = sin (a); return mat3 (vec3 (1., .0, .0), vec3 (.0, c, s), vec3 (.0, -s, c));}
	mat3 rotY (in float a) {float c = cos(a); float s = sin (a); return mat3 (vec3 (c, .0, s), vec3 (.0, 1., .0), vec3 (-s, .0, c));}
	mat3 rotZ (in float a) {float c = cos(a); float s = sin (a); return mat3 (vec3 (c, s, .0), vec3 (-s, c, .0), vec3 (.0, .0, 1.));}
	mat2 rot2d (in float a) {float c = cos(a); float s = sin (a); return mat2 (vec2 (c, s), vec2 (-s, c));}

	vec4 gradient (float v) {
	    float steps = 2.;
	    float step = 1. / steps;
	    vec4 col = green;

	    if (v >= .0 && v < step) {
	        col = mix (green, blue, v * steps);
	    } else if (v >= step && v < 2.0 * step) {
	        col = mix (blue, red, (v - step) * steps);
	    }
	    
	    return col;
	}

	vec3 opRepeat (in vec3 p, in vec3 size) {return mod (p, 2. * size) - size;}
	float sdTorus (in vec3 p, in vec2 t) { vec2 q = vec2 (length (p.xz) - t.x, p.y); return length (q) - t.y; }

	float sdTruchet (in vec3 p, in vec3 t)
	{
	    float offset = t.z;
	    vec3 p1 = vec3 (p - vec3 (offset, offset, .0)) * rotX (radians (90.));
	    vec3 p2 = vec3 (p - vec3 (.0, -offset, offset)) * rotZ (radians (90.));
	    vec3 p3 = vec3 (p - vec3 (-offset, .0, -offset)) * rotY (radians (90.));
	    
	    float t1 = sdTorus (p1, t.xy);
	    float t2 = sdTorus (p2, t.xy);
	    float t3 = sdTorus (p3, t.xy);

	    return min (t1, min (t2, t3));
	}

	float scene (in vec3 p)
	{
	    vec2 mouse = iMouse.xy;
	    if (iMouse.xy == vec2(.0)) mouse.xy = vec2 (212., 192.);
	    mat3 rot = rotX (radians (180. + mouse.y / iResolution.y * 360.)) * rotY (radians (-180. + mouse.x / iResolution.x * 360.));
		p *= rot;
	
		p.x -= iTime * .25;

	    vec3 cellParam = vec3 (.5, .07, .5);

	    float selector = fract(sin(dot(floor(p) + 13.37, vec3(7., 157., 113.)))*43758.5453);

	    if (selector > .75) {
	        p = p;
	    } else if (selector > .5) {
	    	p = p.yzx;
	    } else if (selector > .25) {
		    p = p.zxy;
	    }

	    float d = sdTruchet (opRepeat (p, vec3 (.5)), cellParam);

		return d;
	}

	float trace (in vec3 ro, in vec3 rd, out int iter)
	{
	    float t = .0;

	    for (int i = 0; i < MAX_ITER; i++)
	    {
	        iter = i;
	        vec3 p = ro + t * rd;
	        float d = scene (p);
	        if (d < EPSILON) break;
	        t += d * STEP_SIZE;
	    }

	    return t;
	}

	vec3 normal (in vec3 p)
	{
	    vec3 e = vec3(.0001, .0, .0);
	    float d = scene (p);
	    vec3 n = vec3 (scene (p + e.xyy) - d,
	                   scene (p + e.yxy) - d,
	                   scene (p + e.yyx) - d);
	    return normalize(n);
	}

	vec3 shade (in vec3 ro, in vec3 rd, in float d)
	{
	    vec3 p = ro + d * rd;

	    vec3 ambColor = vec3 (.1, .05, .05);
	    vec3 diffColor = vec3 (1.9, 1.4, 1.2);
	    vec3 specColor = vec3 (.95, .85, .85);
	    float shininess = 60.;
	    float ambient = .75;

	    vec3 lightPos = p + vec3 (cos (iTime) * .5, .5, sin (iTime) * .5);
	    vec3 lightDir = normalize (vec3 (lightPos - p));
	    vec3 nor = normal (p);
	    vec3 ref = normalize (reflect (rd, nor));

	    float diffuse = max (dot (lightDir, nor), .0);
	    float specular = pow (clamp (dot (ref, lightDir), .0, 1.), shininess);

	    return ambient * ambColor + diffuse * diffColor + specular * specColor;
	}

	void main ()
	{
		vec2 uvRaw = fragCoord.xy;
		vec2 uv = uvRaw;
	    uv = uv * 2. - 1.;
	    uv.x *= iResolution.x / iResolution.y;

	    vec3 ro = vec3 (.0, .0, -.75);
	    vec3 rd = normalize (vec3 (uv, 1.) - ro);

	    rd.xz *= rot2d (cos (iTime) * .075);
	    rd.xy *= rot2d (sin (iTime) * .15);

	    int iter = 0;
	    float d = trace (ro, rd, iter);
	    float depth = float (iter) / float (MAX_ITER);
	    vec3 cc = gradient (depth).rgb;
	    float fog = 1. / (1. + d * d * .1);
	    vec3 c = fog * shade (ro, rd, d);

		c = c / (1. + c);
	    c = .2 * c + .8 * sqrt (c);
	    c *= vec3 (.9, .8, .7);
	    c *= .2 + .8 * pow (16. * uvRaw.x * uvRaw.y * (1. - uvRaw.x) * (1. - uvRaw.y), .3);

	    if (iMouse.x / iResolution.x < .5) {
			fragColor = vec4(c, 1.);
	    } else {
			fragColor = vec4(cc, 1.);
	    }
	}
);

const char frag3DGlassNoise[] = GLSL(
    // the uniform inputs taken over from shadertoy.com
    uniform vec3 iResolution;
    uniform float iTime;
    uniform float iTimeDelta;
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

	vec3 hash33(vec3 p)
	{     
		float n = sin(dot(p, vec3(7, 157, 113)));    
		return fract(vec3(2097152, 262144, 32768)*n); 
	}

	float map(vec3 p)
	{
		p = (cos(p*.315*2.5 + sin(p.zxy*.875*2.5 + iTime * .5)));
		float n = length(p);
		p = sin(p*6. + cos(p.yzx*6.));
		return n - 1. - abs(p.x*p.y*p.z)*.05;
	}

	vec3 calcNormal(in vec3 p, float d)
	{
		const vec2 e = vec2(0.01, 0);
		return normalize(vec3(d - map(p - e.xyy), d - map(p - e.yxy),	d - map(p - e.yyx)));
	}

	void main()
	{

	    // normalize, center and aspect-correct texture-coordinates
	    vec2 uv = fragCoord.xy;
	    uv = uv * 2 - 1;
	    uv *= vec2 (iResolution.x / iResolution.y, 1.);

	    vec3 rd = normalize(vec3(uv, (1.-dot(uv, uv)*.5)*.5));
	    vec3 ro = vec3(0., 0., iTime*1.5);
	    vec3 col = vec3(0);
	    vec3 sp;
	    vec3 sn;
	    vec3 lp;
	    vec3 ld;
	    vec3 rnd;

	    vec2 a = sin(vec2(1.5707963, 0) + iTime*0.375);
	    rd.xz = mat2(a, -a.y, a.x)*rd.xz;    
	    rd.xy = mat2(a, -a.y, a.x)*rd.xy; 
	    
	    lp = vec3(0, 1, 4);
	    lp.xz = mat2(a, -a.y, a.x)*lp.xz;    
	    lp.xy = mat2(a, -a.y, a.x)*lp.xy; 
	    lp += ro;
    
	    rnd = hash33(rd+311.);
		float t = length(rnd)*.2;
		float layers = 0.;
		float d;
		float aD;
		float lDist;
		float s;
		float l;
	    float thD = .0125;
		for(float i=0.; i<64.; i++)	{    
	        if(layers>31. || dot(col, vec3(.299, .587, .114)) > 1. || t>16.) break;

	        sp = ro+rd*t;		
	        d = map(sp);
	        aD = (thD-abs(d)*31./32.)/thD;

	        if(aD>0.) {
	            sn = calcNormal(sp, d)*sign(d);
	            ld = (lp - sp);
	            lDist = max(length(ld), .001);
	            ld /= lDist;
	            s = pow(max(dot(reflect(-ld, sn), -rd), 0.), 8.);
	            l = max(dot(ld, sn), 0.);
	            col += ((l + .1) + vec3(.5, .7, 1)*s)*aD/(1. + lDist*0.25 + lDist*lDist*0.05)*.2;            
	            layers++;            
	        }
	        t += max(abs(d)*.75, thD*.25);
		}
		t = min(t, 16.);
     
	    col = mix(col, vec3(0), 1.-exp(-0.025*t*t));
	    uv = abs(fragCoord.xy/iResolution.xy - .5);
	    col = mix(col, vec3(min(col.x*1.5, 1.), pow(col.x, 2.5), pow(col.x, 12.)).yxz,
	               min( dot(pow(uv, vec2(4.)), vec2(1))*8., 1.));

		col = mix(col, col.zxy, dot(sin(rd*5.), vec3(.166)) + 0.166);    
		fragColor = vec4( sqrt(clamp(col, 0., 1.)), 1.0 );
	}
);

const char fragRadialBlur[] = GLSL(
    // the uniform inputs taken over from shadertoy.com
    uniform vec3 iResolution;
    uniform float iTime;
    uniform float iTimeDelta;
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

    const int SAMPLES = 64;
    const float STRENGTH = -0.125;

    void main ()
    {
        vec2 uv = fragCoord.xy;
        uv.y = 1. - uv.y;
        vec2 c = iMouse.xy / iResolution.xy;
        vec2 dir = uv - c;
        vec4 color = vec4 (.0);
        vec4 sum = vec4 (.0);
        float scale = .0;

        for (int i = 0; i < SAMPLES; ++i) {
            scale = (float (i) / float (SAMPLES)) * STRENGTH;
            color = texture2D (iChannel0, uv + dir * scale);
            sum += (color * color);
        }
        sum /= float (SAMPLES);

        fragColor = sqrt (sum);    
    }
);

const char fragEye[] = GLSL(
	// the uniform inputs taken over from shadertoy.com
	uniform vec3 iResolution;
	uniform float iTime;
    uniform float iTimeDelta;
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

	// map 'value' from range a..b to p..q
	float map (float a, float b, float p, float q, float value)
	{
	    float vn = (value - a) / (b - a);
	    float delta = q - p;
		return p + vn * delta;
	}

	float hash (float f)
	{
		return fract (sin (f) * 45785.5453);
	}

	float noise (vec2 p)
	{
	    vec2 u = floor (p);
	    vec2 v = fract (p);
	    
	    v = v * v * (3. - 2. * v);

	    float n = u.x + u.y * 57.;
	    float a = hash (n);
	    float b = hash (n + 1.);
	    float c = hash (n + 57.);
	    float d = hash (n + 58.);

	    float result = mix (mix (a, b, v.x),
	                        mix (c, d, v.x),
	                        v.y);

	    return result;
	}

	mat2 rot (float degree)
	{
		float r = radians (degree);
	    float c = cos (r);
	    float s = sin (r);
	    mat2 m = mat2 (vec2 ( c, s),
	                   vec2 (-s, c));

	    return m;
	}

	float fbm (vec2 p)
	{
		mat2 m1 = rot (5.1);
		mat2 m2 = rot (-4.2);
		mat2 m3 = rot (3.5);

	    float result = .0;
	    result = 0.5 * noise (p);
	    p *= m1 * 2.02;
	    result += 0.25 * noise (p);
	    p *= m2 * 2.03;
	    result += 0.125 * noise (p);
	    p *= m3 * 2.04;
	    result += 0.0625 * noise (p);
	    result /= 0.9375;

	    return result;
	}

	void main ()
	{
	    // normalize, center and aspect-correct texture-coordinates
	    vec2 uv = fragCoord.xy;
	    uv = uv * 2 - 1;
	    uv *= vec2 (iResolution.x / iResolution.y, 1.);

	    // cartesian to polar coordinate conversion
	    float r = dot (uv, uv);
	    float a = atan (uv.x, uv.y);

	    // iris
	    vec3 col = mix (vec3 (1.), vec3 (.0, .35, .45), smoothstep (.1, .2, .95 - length (uv)));

	    // yellow-ish ring around pupil
	    float f = smoothstep (.1, .5, .75 - length (uv));
		col = mix (col, vec3 (.9, .5, .1), f);

	    // stripe-pattern
	    if (r < .675) {
	        // domain distortion
	        a += .125 * fbm (12.*uv);

	        // bright parts
	        f = fbm (vec2 (10. * a, 5. * r));
	        col = mix (col, vec3 (.8, .75, .85), f);

	        // dark parts
	        f = .25 * smoothstep (.25, .95, fbm (vec2 (9. * a, 6. * r))); 
	        col = mix (col, vec3 (.1, .15, .05), f);
	    }

	    // dark tint overlay
	    f = smoothstep (.1, .8, r);
	    col *= 1. - .5 * f;

	    // pupil
	    f = smoothstep (.1, .2, .4 - length (uv));
		col = mix (col, vec3 (0.), f);

	    // fake highlight
	    f = .85 * smoothstep (.1, .8, .75 - length (uv + vec2 (-.15)));
	    col = mix (col, vec3 (.9), f);

	    // put final color to "screen"
	    fragColor = vec4 (col, 1.);
	}
);

const char frag4DJulia[] = GLSL(
	uniform vec3 iResolution;
	uniform float iTime;
    uniform float iTimeDelta;
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

	const float ESCAPE_DISTANCE      = 10.;
	const float BOUNDING_DOMAIN_SIZE = 3.;
	const float DEL                  = .00001; 
	const int MAX_ITERATIONS         = 9;
	const int INTERSECTION_DEPTH     = 96;
	const float EPSILON              = .003;
	const bool SHADOWS               = true;

	vec4 quatMult (vec4 q1, vec4 q2)
	{
		vec4 r;
		r.x   = q1.x*q2.x - dot (q1.yzw, q2.yzw);
		r.yzw = q1.x*q2.yzw + q2.x*q1.yzw + cross (q1.yzw, q2.yzw);
		return r;
	}

	vec4 quatSq (vec4 q)
	{
		vec4 r;
		r.x   = q.x*q.x - dot (q.yzw, q.yzw);
		r.yzw = 2.*q.x*q.yzw;
		return r;
	}

	void iterateIntersect (inout vec4 q,
	                       inout vec4 qp,
	                       vec4 c)
	{
	    for (int i = 0; i < MAX_ITERATIONS; i++) {
		    qp = 2.0 * quatMult(q, qp);
	    	q = quatSq (q) + c;
	    	if (dot (q, q) > ESCAPE_DISTANCE) {
	            break;
	        }
	    }
	}

	vec3 normal (vec3 p, vec4 c)
	{
		vec3 N;
	    vec4 qP = vec4 (p, .0);
		float gradX;
		float gradY;
		float gradZ;
		vec4 gx1 = qP - vec4 (DEL, .0, .0, .0);
		vec4 gx2 = qP + vec4 (DEL, .0, .0, .0);
	    vec4 gy1 = qP - vec4 (.0, DEL, .0, .0);
	    vec4 gy2 = qP + vec4 (.0, DEL, .0, .0);
	    vec4 gz1 = qP - vec4 (.0, .0, DEL, .0);
	    vec4 gz2 = qP + vec4 (.0, .0, DEL, .0);

	    for (int i = 0; i < MAX_ITERATIONS; i++) {
			gx1 = quatSq (gx1) + c;
			gx2 = quatSq (gx2) + c;
			gy1 = quatSq (gy1) + c;
			gy2 = quatSq (gy2) + c;
			gz1 = quatSq (gz1) + c;
			gz2 = quatSq (gz2) + c;
		}

		gradX = length (gx2) - length (gx1);
		gradY = length (gy2) - length (gy1);
		gradZ = length (gz2) - length (gz1);
		N = normalize (vec3 (gradX, gradY, gradZ));
		return N;
	}

	float intersectQJulia (inout vec3 ro,
	                       inout vec3 rd,
	                       vec4 c,
	                       float epsilon)
	{
		float dist;
	    float dummy = .0;
	    for (int i = 0; i < INTERSECTION_DEPTH; ++i)
		{
			vec4 z = vec4 (ro, .0);
			vec4 zp = vec4 (1., .0, .0, .0);
			iterateIntersect (z, zp, c);

			float normZ = length (z);
			dist = .5 * normZ * log (normZ) / length (zp);
			ro += rd * dist;
	        if (dist < epsilon || dot (ro, ro) > BOUNDING_DOMAIN_SIZE) {
		        break;
	        }
		}

		return dist;
	}

	vec3 shade (vec3 light, vec3 eye, vec3 pt, vec3 N)
	{
		vec3 diffuse = vec3 (1., .45, .25);
		const float specularExponent = 10.;
		const float specularity = .45;
		vec3 L = normalize (light - pt);
	    vec3 E = normalize (eye - pt);
		float NdotL = dot (N, L);
		vec3 R = L - 2. * NdotL * N;
		diffuse += abs (N) * .3;

	    return diffuse * max (NdotL, .0) + specularity * pow (max (dot (E,R),.0), specularExponent);
	}

	vec3 intersectSphere (vec3 ro, vec3 rd)
	{
		float B;
		float C;
		float d;
		float t0;
		float t1;
		float t;
		B = 2. * dot (ro, rd);
		C = dot (ro, ro) - BOUNDING_DOMAIN_SIZE;
		d = sqrt (B * B - 4. * C);
		t0 = (-B + d) * .5;
		t1 = (-B - d) * .5;
		t = min (t0, t1);
		ro += t * rd;
		return ro;
	}

	mat3 camera (vec3 ro, vec3 ta, float cr)
	{
		vec3 cw = normalize (ta - ro);
		vec3 cp = vec3 (sin (cr), cos (cr), .0);
		vec3 cu = normalize (cross (cw, cp));
		vec3 cv = normalize (cross (cu, cw));
	    return mat3 (cu, cv, cw);
	}

	void main ()
	{
		vec2 uvRaw = fragCoord.xy;
		vec2 uv = uvRaw;
		uv = uv * 2 - 1;
		uv *= vec2 (iResolution.x / iResolution.y, 1.);
		uvRaw *= (iResolution.x / iResolution.y, 1.);

		vec2 mouse = iMouse.xy / iResolution.xy;
	    vec3 eye = vec3 (.5 + 2.5 * cos (6. * mouse.x),
						 -2. + 4. * sin (mouse.y),
						 .5 + 2.5 * sin (6. * mouse.x));
	    vec3 aim = vec3 (.0, .0, .0);
	    mat3 ca = camera (eye, aim, .0);
	    float u = (fragCoord.x * 2. - 1.) * iResolution.z;
	    float v = fragCoord.y * 2. - 1.;
		vec3 ro = eye;
		vec3 rd = normalize (ca * vec3 (u, -v, 2.));

	    vec4 mu = vec4 (-.2, .6, .5 * cos (iTime), .125 * sin (iTime));
	    vec3 light1 = vec3 (cos (iTime) * 2., 12., sin (iTime) * 15.);
	    vec3 light2 = vec3 (cos (iTime) * (-3.), -6., sin (iTime) * (-10.));
		const vec4 backgroundColor = vec4 (.3, .3, .3, .0);
		vec4 color;

		color = backgroundColor;
		rd = normalize (rd);
		ro = intersectSphere (ro, rd);

	    float dist = intersectQJulia (ro, rd, mu, EPSILON);
		if(dist < EPSILON) {
			vec3 N = normal (ro, mu);
			color.rgb = shade (light1, rd, ro, N);
			color.rgb += shade (light2, rd, ro, N);
			color.a = 1.;
			if (SHADOWS) {
				vec3 L = normalize (light1 - ro);
				ro += N * EPSILON * 2.;

				dist = intersectQJulia (ro, L, mu, EPSILON);
	            if (dist < EPSILON) {
					color.rgb *= .4;
	            }
			}
		}

		// gamma-correction, tint, vingette
		color.rgb = .2 * color.rgb + .8 * sqrt (color.rgb);
		color.rgb *= vec3 (.9, .8, .7);
	    color.rgb *= .2 + .8 * pow (16. * uvRaw.x * uvRaw.y * (1. - uvRaw.x) * (1. - uvRaw.y), .3);

		fragColor = color;
	}
);

const char frag2DRaymarch[] = GLSL(
	// the uniform inputs taken over from shadertoy.com
	uniform vec3 iResolution;
	uniform float iTime;
    uniform float iTimeDelta;
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

	const vec3 white   = vec3 (1.0, 1.0, 1.0);
	const vec3 black   = vec3 (0.0, 0.0, 0.0);

	vec2 opRepeat (vec2 p, vec2 offset)
	{
		return mod (p, offset) - .5 * offset;
	}

	vec2 opRot (vec2 p, float degree)
	{
	    float angle = radians (degree);
	    float c = cos (angle);
	    float s = sin (angle);
	    mat2 mat = mat2 (vec2 (c, s), vec2 (-s, c));

	    return mat * p;
	}

	vec2 opTwist (vec2 p, float angle)
	{
	    float rad = radians (angle * p.y * p.x * 5.);
	    float c = cos (rad);
	    float s = sin (rad);
	    mat2 mat = mat2 (vec2 ( c, s),
	                     vec2 (-s, c));

	    return p * mat;
	}

	float disc (vec2 p, float r)
	{
	    return smoothstep (.0, 10. / iResolution.x, length (p) - r);
	}

	float rect (vec2 p, vec2 q)
	{
	    vec2 d = abs (p) - q;
	    return smoothstep (.0, 10. / iResolution.x, length (max (d, .0)));
	}

	float rrect (vec2 p, vec2 q, float r)
	{
	    vec2 d = abs (p) - q;
	    return smoothstep (.0, 10. / iResolution.x,
	    				   length (max (d + vec2 (r), .0)) - r);
	}

	// cartesian- to polar-coordinate conversion, resulting .x holds angle in
	// radians and resulting .y holds radius/distance from origin
	vec2 polar (vec2 p)
	{
		return vec2 (atan (p.y, p.x), dot (p, p));
	}

	float ddisc (vec2 p, float r)
	{
	    vec2 pol = polar (p);
	    return smoothstep (.0, 10. / iResolution.x,
	    				   length (p) - r + .025 * sin (10. * (pol.x + iTime)));
	}

	void main ()
	{
	    vec2 uv = fragCoord.xy;
	    uv = uv * 2 - 1;
	    uv *= vec2 (iResolution.x / iResolution.y, 1.0);

	    // take a "copy" of the texture coordinate 
		vec2 pt = opRot (uv, 360. * sin (.1 * iTime));
		pt *= 1. + .25 * cos (iTime);

	    // circle and rounded rectangle, repeat and distort them... SDF-style
	    float f = ddisc (opRepeat (pt + vec2 (-.15, .875), vec2 (-1.7)), .4);
	    f = min (f, rrect (opRot (opTwist (opRepeat (vec2 (.3, -.4) + pt,
	    											 vec2 (.85)),
	                                       75.),
	                              15. * iTime),
	                       vec2 (.3, .2),
	                       .05));

	    vec3 color = mix (white, black, f);
		fragColor = vec4 (color , 1.);
	}
);

const char fragMetaBalls[] = GLSL(
	// the uniform inputs taken over from shadertoy.com
	uniform vec3 iResolution;
	uniform float iTime;
    uniform float iTimeDelta;
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

	const float PI     = 3.14159265358979323846;
	const vec4 red     = vec4 (1.0, 0.0, 0.0, 1.0);
	const vec4 green   = vec4 (0.0, 1.0, 0.0, 1.0);
	const vec4 blue    = vec4 (0.0, 0.0, 1.0, 1.0);
	const vec4 white   = vec4 (1.0, 1.0, 1.0, 1.0);
	const vec4 orange  = vec4 (1.0, 0.4, 0.125, 1.0);
	const vec4 black   = vec4 (0.0, 0.0, 0.0, 1.0);
	const vec4 cyan    = vec4 (0.0, 1.0, 1.0, 1.0);
	const vec4 magenta = vec4 (1.0, 0.0, 1.0, 1.0);
	const vec4 yellow  = vec4 (1.0, 1.0, 0.0, 1.0);

	// metaballs, abusing vec3 to store x (r), y (g) and radius(b) of metaball
	vec3 ball0 = vec3 (-.5, .5, .05);
	vec3 ball1 = vec3 (.3, .2, .06);
	vec3 ball2 = vec3 (.6, -.7, .13);
	vec3 ball3 = vec3 (1., .4, .1);
	vec3 ball4 = vec3 (-.9, .4, .02);
	vec3 ball5 = vec3 (.4, -.4, .1);
	vec3 ball6 = vec3 (-1., .8, .09);

	// this is to make a metaball move around a little
	vec2 getPos (vec2 pos, float tick)
	{
	    float c = .2 * cos (tick);
	    float s = .2 * sin (tick);
	    vec2 offset = vec2 (c, s);
		return pos + offset;
	}

	float getIso (vec3 ball, vec2 curr, float tick)
	{
		return ball.b / distance (curr, getPos (ball.rg, tick));
	}

	vec4 gradient (float v) {
	    float steps = 7.;
	    float step = 1. / steps;
	    vec4 col = green;

	    if (v >= .0 && v < step) {
	        col = mix (yellow, orange, v * steps);
	    } else if (v >= step && v < 2.0 * step) {
	        col = mix (orange, red, (v - step) * steps);
	    } else if (v >= 2.0 * step && v < 3.0 * step) {
	        col = mix (red, magenta, (v - 2.0 * step) * steps);
	    } else if (v >= 3.0 * step && v < 4.0 * step) {
	        col = mix (magenta, cyan, (v - 3.0 * step) * steps);
	    } else if (v >= 4.0 * step && v < 5.0 * step) {
	        col = mix (cyan, blue, (v - 4.0 * step) * steps);
	    } else if (v >= 5.0 * step && v < 6.0 * step) {
	        col = mix (blue, green, (v - 5.0 * step) * steps);
	    }

	    return col;
	}

	void main ()
	{
	    vec2 uv = fragCoord.xy;
	    uv = uv * 2 - 1;
	    uv *= vec2 (iResolution.x / iResolution.y, 1.0);
	    float rad = -.125 * iTime * PI;
	    float c = cos (rad);
	    float s = sin (rad);
	    uv *= mat2 (vec2 (c, -s), vec2 (s, c));

	    float col = getIso (ball0, uv, .5 * iTime);
	    col += getIso (ball1, uv, -.6 * iTime);
	    col += getIso (ball2, uv, .7 * iTime);
	    col += getIso (ball3, uv, -.8 * iTime);
	    col += getIso (ball4, uv, .6 * iTime);
	    col += getIso (ball5, uv, -.5 * iTime);
	    col += getIso (ball6, uv, .7 * iTime);

		fragColor = gradient (col);
	}
);

const char fragFractalExplorer[] = GLSL(
	// the uniform inputs taken over from shadertoy.com
	uniform vec3 iResolution;
	uniform float iTime;
    uniform float iTimeDelta;
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

	const float PI = 3.14159265358979323846;
	const vec4 red     = vec4 (1.0, 0.0, 0.0, 1.0);
	const vec4 green   = vec4 (0.0, 1.0, 0.0, 1.0);
	const vec4 blue    = vec4 (0.0, 0.0, 1.0, 1.0);
	const vec4 white   = vec4 (1.0, 1.0, 1.0, 1.0);
	const vec4 orange  = vec4 (1.0, 0.4, 0.125, 1.0);
	const vec4 black   = vec4 (0.0, 0.0, 0.0, 1.0);
	const vec4 cyan    = vec4 (0.0, 1.0, 1.0, 1.0);
	const vec4 magenta = vec4 (1.0, 0.0, 1.0, 1.0);
	const vec4 yellow  = vec4 (1.0, 1.0, 0.0, 1.0);
	const float MAX_ITER = 64.;
	const float LENGTH_LIMIT = 5.;
	const bool SHOW_ORBIT = true;
	const float THICKNESS = 0.0075;

	vec4 line (vec2 p, vec2 a, vec2 b, vec4 c, float thickness)
	{
	    vec2 pa = -p - a;
	    vec2 ba = b - a;
	    float h = clamp (dot (pa, ba) / dot (ba, ba), 0.0, 1.0);
	    float d = length (pa - ba * h);
	    
	    return c * clamp (((1.0 - d) - (1.0 - thickness)) * 100.0, 0.0, 1.0);
	}

	vec4 gradient (float v) {
	    float steps = 7.;
	    float step = 1. / steps;
	    vec4 col = black;

	    if (v >= .0 && v < step) {
	        col = mix (yellow, orange, v * steps);
	    } else if (v >= step && v < 2.0 * step) {
	        col = mix (orange, red, (v - step) * steps);
	    } else if (v >= 2.0 * step && v < 3.0 * step) {
	        col = mix (red, magenta, (v - 2.0 * step) * steps);
	    } else if (v >= 3.0 * step && v < 4.0 * step) {
	        col = mix (magenta, cyan, (v - 3.0 * step) * steps);
	    } else if (v >= 4.0 * step && v < 5.0 * step) {
	        col = mix (cyan, blue, (v - 4.0 * step) * steps);
	    } else if (v >= 5.0 * step && v < 6.0 * step) {
	        col = mix (blue, green, (v - 5.0 * step) * steps);
	    }

	    return col;
	}

	vec4 calcMandel (vec2 c)
	{
	    vec2 z = vec2 (.0);
	    float iter = .0;

	    for (float i = 0.; i < MAX_ITER; i += 1.) {
	        z = mat2 (z, -z.y, z.x) * z + c;
	        if (length (z) > LENGTH_LIMIT && iter == .0) {
	            iter = i;
	        }
	    }

	    return length (z) <= LENGTH_LIMIT ? vec4 (0) : gradient (iter / MAX_ITER);
	}

	vec4 mandel (vec2 p, vec2 size)
	{
	    // ordered 2x2-grid super-sampling
	    return (  calcMandel (p + size * vec2 (-.5, -.5))
	            + calcMandel (p + size * vec2 ( .5, -.5))
	            + calcMandel (p + size * vec2 ( .5,  .5))
	            + calcMandel (p + size * vec2 (-.5,  .5))) / 4.;
	}

	vec4 calcJulia (vec2 p, vec2 c)
	{
	    vec2 z = p;
	    float iter = .0;

	    for (float i = 0.; i < MAX_ITER; i+= 1.) {
	        z = mat2 (z, -z.y, z.x) * z + c;
	        if (length (z) > LENGTH_LIMIT && iter == .0) {
	            iter = i;
	        }
	    }
	    return length (z) <= LENGTH_LIMIT ? vec4 (0) : gradient (iter / MAX_ITER);
	}

	vec4 julia (vec2 p, vec2 size, vec2 c)
	{
	    // ordered 2x2-grid super-sampling
	    return (  calcJulia (p + size * vec2 (-.5, -.5), c)
	            + calcJulia (p + size * vec2 ( .5, -.5), c)
	            + calcJulia (p + size * vec2 ( .5,  .5), c)
	            + calcJulia (p + size * vec2 (-.5,  .5), c)) / 4.;
	}

	vec4 morbit (vec2 p, vec2 s, float thickness)
	{
	    vec4 result = vec4 (.0);
	    vec2 z = vec2 (.0);
	    vec2 zNew = vec2 (.0);
	    vec2 c = s;

	    for (float i = .0; i < MAX_ITER; i+= 1.) {
	        zNew = mat2 (z, -z.y, z.x) * z + c;
		    result += line (p, vec2 (-z.x, z.y), vec2 (-zNew.x, zNew.y), vec4 (1.), thickness);
	        z = zNew;
	    }
	    return result;
	}

	vec4 jorbit (vec2 p, vec2 s, float thickness)
	{
	    vec4 result = vec4 (.0);
	    vec2 z = s;
	    vec2 zNew = vec2 (.0);
	    vec2 c = vec2(.0);

	    for (float i = .0; i < MAX_ITER; i+= 1.) {
	        zNew = mat2 (z, -z.y, z.x) * z + c;
		    result += line (p, vec2 (-z.x, z.y), vec2 (-zNew.x, zNew.y), vec4 (1.), thickness);
	        z = zNew;
	    }
	    return result;
	}

	void main ()
	{
	    vec2 res = iResolution.xy;
	    vec2 uv = fragCoord;
	    vec2 nuv = -1. + 2. * uv;
	    float aspect = iResolution.x / iResolution.y;
	    nuv.y /= aspect;

	    float s = 3.;
	    vec2 moffset = s * vec2 (1./4., .0);
	    vec2 joffset = s * vec2 (1./6., .0);
	    vec2 pmandel = vec2 (nuv) - moffset;
	    vec2 pjulia = vec2 (nuv) + joffset;
	    float t = iTime;

	    vec2 m = vec2 (iMouse.x / iResolution.x, iMouse.y / iResolution.y);
	    vec2 nc = -1. + 2. * m;
	    vec2 mc = nc - moffset;
	    mc *= s;
	    mc.y /= aspect;
	    mc.y *= 1.;

	    if (nuv.x > .0) {
	    	fragColor = mandel (s * pmandel , s/res);
	    } else {
	    	fragColor = julia (s * pjulia , s/res, mc);
	    }

	    if (SHOW_ORBIT) {
	        if (nuv.x > .0) {
			    fragColor += morbit (s * pmandel, mc, THICKNESS);
	        }
	    }
	}
);

const char fragJellyBoxes[] = GLSL(
	// the uniform inputs taken over from shadertoy.com
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

	float hash (float f)
	{
		return fract (sin (f) * 45734.5453);
	}

	float noise3d (vec3 p)
	{
	    vec3 u = floor (p);
	    vec3 v = fract (p);
	    
	    v = v * v * (3. - 2. * v);

	    float n = u.x + u.y * 57. + u.z * 113.;
	    float a = hash (n);
	    float b = hash (n + 1.);
	    float c = hash (n + 57.);
	    float d = hash (n + 58.);

	    float e = hash (n + 113.);
	    float f = hash (n + 114.);
	    float g = hash (n + 170.);
	    float h = hash (n + 171.);

	    float result = mix (mix (mix (a, b, v.x),
	                             mix (c, d, v.x),
	                             v.y),
	                        mix (mix (e, f, v.x),
	                             mix (g, h, v.x),
	                             v.y),
	                        v.z);

	    return result;
	}

	mat3 rotY (in float angle)
	{
	    float rad = radians (angle);
	    float c = cos (rad);
	    float s = sin (rad);

	    mat3 mat = mat3 (vec3 ( c, .0, -s),
	                     vec3 (.0, 1,  .0),
	                     vec3 ( s, .0,  c));

	    return mat;
	}

	mat3 rotZ (in float angle)
	{
	    float rad = radians (angle);
	    float c = cos (rad);
	    float s = sin (rad);

	    mat3 mat = mat3 (vec3 (  c,  -s, 0.0),
	                     vec3 (  s,   c, 0.0),
	                     vec3 (0.0, 0.0, 1.0));

	    return mat;
	}

	float fbm (vec3 p)
	{
		mat3 m1 = mat3 (rotZ (23.4));
		mat3 m2 = mat3 (rotZ (45.5));
		mat3 m3 = mat3 (rotZ (77.8));

	    float result = .0;
	    result = 0.5 * noise3d (p);
	    p *= m1 * 2.02;
	    result += 0.25 * noise3d (p);
	    p *= m2 * 2.03;
	    result += 0.125 * noise3d (p);
	    p *= m3 * 2.04;
	    result += 0.0625 * noise3d (p);
	    result /= 0.9375;

	    return result;
	}

	float sphere (vec3 p, float size)
	{
		return length (p) - size;
	}

	float box (vec3 p, float size)
	{
		return length (max (abs (p) - vec3 (size), .0)) - .2*size;
	}

	float plane (vec3 p)
	{
	    return p.y + .25;
	}

	float func (vec3 p)
	{
		float f1 = 16.;
		float f2 = 7.;

		return pow (p.x, f1) +
			   pow (p.y, f1) +
			   pow (p.z, f1) -
			   cos (f2 * p.x) -
			   cos (f2 * p.y) -
			   cos (f2 * p.z);
	}

	float opRepeat (inout float v, float w)
	{
	    float halfSize = w / 2.;
	    v = mod(v + halfSize, w) - halfSize;

	    return .0;
	}

	float opBend (inout vec3 p, float deg)
	{
	    float rad = radians (deg);
	    float c = cos (rad * p.y);
	    float s = sin (rad * p.y);
	    mat2  m = mat2 (c, -s, s, c);
	    p = vec3 (m * p.xy, p.z);
	    return .0;
	}

	float opRepeat2 (inout vec2 v, float w)
	{
	    float halfSize = w / 2.;
	    v = mod(v + vec2 (halfSize), vec2 (w)) - vec2 (halfSize);

	    return .0;
	}

	float displace (vec3 p, int flag)
	{
	    float result = 1.;

	    if (flag == 0) {
	        result = .75 * fbm (.75 * (.5 + .25 * cos (iTime)) * p);
	    } else if (flag == 1) {
	        result =  sin (2. * p.x) * cos (2. * p.y) * sin (2. * p.z);
	    } else if (flag == 2) {
	        result = (.5 + .5 * (1. + cos (iTime))) * fbm (p);
	    }

	    return result;
	}

	vec2 map (vec3 p)
	{
		// this let's an index loop through 0, 1, 2 for selecting the object to draw
	    int object = int (floor (4. * (1. + sin (.25 * iTime)) / 2.));

	    float dt = .0;
	    float dp = .0;
		vec3 w = vec3 (.0);
		vec2 d1 = vec2 (.0);
		mat3 m = rotY (20. * iTime) * rotZ (-20. * iTime);

	    // floor
	    vec2 d2 = vec2 (plane (p), 2.);

	    if (object == 0) {
	        // redish cube
	        w = m * vec3 (p + vec3 (.0, -1.5, .0));
	        dt = box (w, 1.125);
	        dp = displace (w, object);
	        d1 = vec2 (dt + dp, 1.);
	    } else if (object == 1) {
	        // blue thingy
	        w = -m * (p + vec3 (.0, -1.5, .0));
	        opBend (w, 42.5 * cos (iTime));
	        dt = sphere (w, 1.);
	        dp = displace (w, object);
	        d1 = vec2 (dt + dp, 3.);
	    } else if (object == 2) {
	        // green blob
	        w = p + vec3 (.0, -1.5, .0);
	        dt = sphere (w, 1.);
	        dp = displace (w, object);
	        d1 = vec2 (dt + dp, 4.);
	    } else if (object == 3) {
	        // green blob
	        w = p + vec3 (.0, -1.5, .0);
	        dt = func (w);
	        dp = .0; //displace (w, object);
	        d1 = vec2 (dt + dp, 5.);	    	
	    }

	    if (d2.x < d1.x) {
	        d1 = d2;
	    }

		return d1;
	}

	vec3 normal (vec3 p)
	{
		vec3 n;
	    vec2 e = vec2 (.001, .0);
	    n.x = map (p + e.xyy).x - map (p - e.xyy).x;
	    n.y = map (p + e.yxy).x - map (p - e.yxy).x;
	    n.z = map (p + e.yyx).x - map (p - e.yyx).x;
	    return normalize (n);
	}

	const int MAX_STEPS = 80;

	vec2 march (vec3 ro, vec3 rd)
	{
	    float pixelSize = 1. / iResolution.x;
	    bool forceHit = true;
	    float infinity = 10000000.0;
	    float t_min = .0000001;
	    float t_max = 1000.0;
	    float t = t_min;
	    vec2 candidate = vec2 (t_min, .0);
	    vec2 candidate_error = vec2 (infinity, .0);
	    float w = .5;
	    float lastd = .0;
	    float stepSize = .0;
	    float sign = map (ro).x < .0 ? -1. : 1.;

	    for (int i = 0; i < MAX_STEPS; i++)
		{
	        float signedd = sign * map (ro + rd * t).x;
	        float d = abs (signedd);
	        bool fail = w > 1. && (d + lastd) < stepSize;

	        if (fail) {
	            stepSize -= w * stepSize;
	            w = .25;
	        } else {
	            stepSize = signedd * w;
	        }

			lastd = d;

	        float error = d / t;
	        if (!fail && error < candidate_error.x) {
	            candidate_error.x = error;
	            candidate.x = t;
	        }

	        if (!fail && error < pixelSize || t > t_max) {
	        	break;
			}

	        candidate_error.y = map (ro + rd * t).y;
	        candidate.y = candidate_error.y;

	        t += stepSize;
	 
		}

	    if ((t > t_max || candidate_error.x > pixelSize) && !forceHit) {
	        return vec2 (infinity, .0);
	    }

		return candidate;
	}

	float shadow (vec3 ro, vec3 rd)
	{
	    float result = 1.;
	    float t = .1;
	    for (int i = 0; i < 32; i++) {
	        float h = map (ro + t * rd).x;
	        if (h < 0.00001) return .0;
	        result = min (result, 8. * h/t);
	        t += h;
	    }

	    return result;
	}

	vec3 boxMaterial (vec3 pos, vec3 nor)
	{
		vec3 col = vec3 (1., .0, .0);
	    float f = fbm (5. * pos);
	    col = mix (col, vec3 (1., .5, .25), f);

	    return col;
	}

	vec3 floorMaterial (vec3 pos, vec3 nor)
	{
	    vec3 col = vec3 (.6, .5, .3);
	    float f = fbm (pos * vec3 (6., .0, .5));
	    col = mix (col, vec3 (.3, .2, .1), f);
	    f = smoothstep (.6, 1., fbm (48. * pos));
	    col = mix (col, vec3 (.2, .2, .15), f);

	    return col;
	}

	void main ()
	{
	    vec2 aspect = vec2 (iResolution.x/ iResolution.y, 1.);
	    vec2 uv = fragCoord.xy;
	    vec2 p = vec2 (-1. + 2. * uv) * aspect;

	    vec3 ro = 3.5 * vec3 (cos (.2 * iTime), 1.25, sin (.2 * iTime));
	    vec3 ww = normalize (vec3 (.0, 1., .0) - ro);
	    vec3 uu = normalize (cross (vec3 (.0, 1., .0), ww));
	    vec3 vv = normalize (cross (ww, uu));
	    vec3 rd = normalize (p.x * uu + p.y * vv + 1.5 * ww);

	    vec2 t = march (ro, rd);

	    vec3 col = vec3 (.8);
	    if (t.y > .5) {
	        vec3 pos = ro + t.x * rd;
	        vec3 nor = normal (pos);
	        vec3 lig = normalize (vec3 (1., .8, .6));
	        vec3 blig = normalize (vec3 (-lig.x, lig.y, -lig.z));
	        vec3 ref = normalize (reflect (rd, nor));

	        float con = 1.;
	        float amb = .5 + .5 * nor.y;
	        float diff = max (dot (nor, lig), .0);
	        float bac = max (.2 + .8 * dot (nor, blig), .0);
	        float sha = shadow (pos, lig);
	        float spe = pow (clamp (dot (ref, lig), .0, 1.), 8.);
	        float rim = pow (1. + dot (nor, rd), 2.5);

	        col  = con  * vec3 (.1, .15, .2);
	        col += amb  * vec3 (.1, .15, .2);
	        col += diff * vec3 (1., .97, .85) * sha;
	        col += spe * vec3 (.9, .9, .9);
	        col += bac;

	        if (t.y == 1.) {
	            col *= boxMaterial (pos, nor);
	        } else if (t.y == 2.) {
	            col *= floorMaterial (pos, nor);
	        } else if (t.y == 3.) {
	            col *= vec3 (.0, .2, .8);
	        } else if (t.y == 4.) {
	            col *= vec3 (.2, .75, .1);
	        } else if (t.y == 5.) {
	            col *= vec3 (.75, .8, .25);
	        }

	        col += .6 * rim * amb;
	        col += .6 * spe * sha * amb;

	        // gamma-correction
	        col = .1 * col + .9 * sqrt (col);

	        // warm tint
	        col *= vec3 (.9, .8, .7);
	    }

	    // vignette
	    col *= .2 + .8 * pow (16. * uv.x * uv.y * (1. - uv.x) * (1. - uv.y), .2);

	    fragColor = vec4(col, 1.);
	}
);

const char fragNoise[] = GLSL(
	// the uniform inputs taken over from shadertoy.com
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

	const float PI = 3.14159265358979323846;
	const vec4 red = vec4 (1.0, 0.0, 0.0, 1.0);
	const vec4 green = vec4 (0.0, 1.0, 0.0, 1.0);
	const vec4 blue = vec4 (0.0, 0.0, 1.0, 1.0);
	const vec4 white = vec4 (1.0);
	const vec4 orange = vec4 (1.0, 0.4, 0.125, 1.0);
	const vec4 black = vec4 (0.0, 0.0, 0.0, 1.0);
	const vec4 cyan = vec4 (0.0, 1.0, 1.0, 1.0);
	const vec4 magenta = vec4 (1.0, 0.0, 1.0, 1.0);
	const vec4 yellow = vec4 (1.0, 1.0, 0.0, 1.0);

	float hash (float x)
	{
		return fract (sin (x) * 43758.5453);
	}

	float noise (vec3 p)
	{
		vec3 u = floor (p);
		vec3 v = fract (p);
	
		v = v * v * (3. - 2. * v);

		float n = u.x + u.y * 57. + u.z * 113.;
	
		float a = hash (n);
		float b = hash (n + 1.);
		float c = hash (n + 57.);
		float d = hash (n + 58.);

		float e = hash (n + 113.);
		float f = hash (n + 114.);
		float g = hash (n + 170.);
		float h = hash (n + 171.);

		float result = mix (mix (mix (a, b, v.x), mix (c, d, v.x), v.y),
			                mix (mix (e, f, v.x), mix (g, h, v.x), v.y),
			                v.z);
		return result;
	}

    mat3 rot (float angle)
    {
        float rad = radians (angle);
        float c = cos (rad);
        float s = sin (rad);

        mat3 mat = mat3 (vec3 ( c, -s, .0),
                         vec3 ( s,  c, .0),
                         vec3 (.0, .0, 1.));

        return mat;
    }

    mat3 m1 = rot (1.1 * iTime);
    mat3 m2 = rot (-1.2 * iTime);
    mat3 m3 = rot (1.3 * iTime);

	float fbm (vec3 p)
	{
		float result = 0.;
		result =  0.5000 * noise (p);
		p *= m1 * 2.02;
		result += 0.2500 * noise (p);
		p *= m2 * 2.03;
		result += 0.1250 * noise (p);
		p *=  m3 * 2.04;
		result += 0.0625 * noise (p);
		result /= 0.9375;

		return result;
	}

	vec3 gradient (float v) {
		float step = 1.0 / 7.0;
		vec4 col = black;

		if (v >= 0.0 && v < step) {
			col = mix (yellow, orange, v * 7.0);
		} else if (v >= step && v < 2.0 * step) {
			col = mix (orange, red, (v - step) * 7.0);
		} else if (v >= 2.0 * step && v < 3.0 * step) {
			col = mix (red, magenta, (v - 2.0 * step) * 7.0);
		} else if (v >= 3.0 * step && v < 4.0 * step) {
			col = mix (magenta, cyan, (v - 3.0 * step) * 7.0);
		} else if (v >= 4.0 * step && v < 5.0 * step) {
			col = mix (cyan, blue, (v - 4.0 * step) * 7.0);
		} else if (v >= 5.0 * step && v < 6.0 * step) {
			col = mix (blue, green, (v - 5.0 * step) * 7.0);
		}

		return col.rgb;
	}

	void main()
	{
		float aspect = iResolution.x / iResolution.y;
		float t = sin (iResolution.x * iResolution.y);
		vec3 p = vec3 (gl_FragCoord.s, gl_FragCoord.t, t);
		vec3 g = gradient (fbm (.02 * p));
		vec3 r = gradient (fbm (.01 * p));
		vec3 b = gradient (fbm (.03 * p));
		float fg = fbm (.02 * p);
		float fr = fbm (.01 * p);
		float fb = fbm (.03 * p);

		vec4 col = vec4 (r * g * b, 1.);
		if (p.x < iResolution.x * .25) {
			col = vec4 (vec3 (fr, fg, fb), 1);
		} else if (p.x < iResolution.x * .5) {
			col = vec4 (r+ g + b, 1);
		} else if (p.x < iResolution.x * .75) {
			col = vec4 (vec3 (fr), 1);
		}

	    gl_FragColor = col;
	}
);

const char fragRaymarch[] = GLSL(
	// the uniform inputs taken over from shadertoy.com
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

	const float PI = 3.14159265358979323846;
	const vec4 red   = vec4 (1., .0, .0, 1.);
	const vec4 green = vec4 (.0, 1., .0, 1.);
	const vec4 blue  = vec4 (.0, .0, 1., 1.);
	const vec4 white = vec4 (1.);
	const vec4 black = vec4 (.0, .0, .0, 1.);
	const vec4 orange = vec4 (1., .5, .25, 1.);
	const float EPSILON = 0.00001;
	const int MAX_STEPS = 64;
	int steps = 0;

	vec4 gradient (float v) {
		float steps = 3.;
		float step = 1. / steps;
		vec4 col = black;

		if (v >= 0.0 && v < step) {
			col = mix (white, blue, v * steps);
		} else if (v >= step && v < 2.0 * step) {
			col = mix (blue, red, (v - step) * steps);
		}

		return col;
	}

	float deg2rad (in float degree)
	{
		return degree * PI / 180.0;
	}

	mat4 trans (vec3 t)
    {
        mat4 mat = mat4 (vec4 (1., .0, .0, .0),
                         vec4 (.0, 1., .0, .0),
                         vec4 (.0, .0, 1., .0),
                         vec4 (t.x, t.y, t.z, 1.));
        return mat;
    }

	mat4 rotX (in float angle)
	{
	    float rad = deg2rad (angle);
	    float c = cos (rad);
	    float s = sin (rad);

	    mat4 mat = mat4 (vec4 (1.0, 0.0, 0.0, 0.0),
	                     vec4 (0.0,   c,   s, 0.0),
	                     vec4 (0.0,  -s,   c, 0.0),
	                     vec4 (0.0, 0.0, 0.0, 1.0));

	    return mat;
	}

	mat4 rotY (in float angle)
	{
	    float rad = deg2rad (angle);
	    float c = cos (rad);
	    float s = sin (rad);

	    mat4 mat = mat4 (vec4 (  c, 0.0,  -s, 0.0),
	                     vec4 (0.0, 1.0, 0.0, 0.0),
	                     vec4 (  s, 0.0,   c, 0.0),
	                     vec4 (0.0, 0.0, 0.0, 1.0));

	    return mat;
	}

	mat4 rotZ (in float angle)
	{
	    float rad = deg2rad (angle);
	    float c = cos (rad);
	    float s = sin (rad);

	    mat4 mat = mat4 (vec4 (  c,   s, 0.0, 0.0),
	                     vec4 ( -s,   c, 0.0, 0.0),
	                     vec4 (0.0, 0.0, 1.0, 0.0),
	                     vec4 (0.0, 0.0, 0.0, 1.0));

	    return mat;
	}

	float smin (float a, float b, float k)
	{
	    float h = clamp (.5 + .5 * (b - a) / k, .0, 1.);
	    return mix (b, a, h) - k * h * (1. - h);
	}

	vec3 opTransf (vec3 p, mat4 m)
	{
		return vec4 (m * vec4 (p, 1.)).xyz;
	}

	float opUnion (float d1, float d2)
	{
		return min (d1, d2);
	}

	float opIntersect (float d1, float d2)
	{
		return max (d1, d2);
	}

	float opSubtract (float d1, float d2)
	{
		return max (-d1, d2);
	}

	vec3 opRepeat (vec3 p, vec3 v)
	{
		return mod (p, v) - .5 * v;
	}

	vec3 opTwistX (vec3 p, float angle)
	{
		float rad = deg2rad (angle * p.x);
		float c = cos (rad);
		float s = sin (rad);

	    mat3 mat = mat3 (vec3 (1.0, 0.0, 0.0),
	                     vec3 (0.0,   c,   s),
	                     vec3 (0.0,  -s,   c));

		return p * mat;
	}

	vec3 opTwistY (vec3 p, float angle)
	{
		float rad = deg2rad (angle * p.y);
		float c = cos (rad);
		float s = sin (rad);

		mat3 mat = mat3 (vec3 (  c, 0.0,  -s),
						 vec3 (0.0, 1.0, 0.0),
						 vec3 (  s, 0.0,   c));

		return p * mat;
	}

	vec3 opTwistZ (vec3 p, float angle)
	{
		float rad = deg2rad (angle * p.z);
		float c = cos (rad);
		float s = sin (rad);

		mat3 mat = mat3 (vec3 (  c,   s, 0.0),
						 vec3 ( -s,   c, 0.0),
						 vec3 (0.0, 0.0, 1.0));

		return p * mat;
	}

	float opBlend (float d1, float d2, float k)
	{
		return smin (d1, d2, k);
	}

	float sphere (vec3 p, float r)
	{
		return length (p) - r;
	}

	float box (vec3 p, vec3 b, float r)
	{
		return length (max (abs (p) - b + vec3 (r), .0)) - r;
	}

	float cylinder (vec3 p, vec3 n)
	{
		return length (p.xz - n.xy) - n.z;
	}

	float plane (vec3 p, vec4 n)
	{
		// n must be normalized
		return dot (p.xyz, n.xyz) + n.w;
	}

	float map (vec3 p)
	{
		float d = .0;
		float d2 = .0;
		float d3 = .0;
		float t = iTime;

		// twisting box with sphere cutout
		mat4 m = trans (vec3 (-2., -1., 4.));
		d = opSubtract (sphere (opTransf (p, m), 1.),
						box (opTwistY (opTransf (p, m),
									   sin (t) * 45.),
							 vec3 (.6, 2., .6), .1));

		m = trans (vec3 (.0, -1., .0));
		d2 = plane (opTransf (p, m), normalize (vec4 (.0, -1., .0, 1.)));
		d = opUnion (d, d2);

        // dice-like things
        m = rotY (50. * t) * trans (vec3 (-3.6, -.5, .5 + 1.5 * sin (t)));
        mat4 m1 = trans (vec3 (-3., -1.45, .0));
        d3 = opBlend (opIntersect (box (opTransf (p, m), vec3 (.55), .1),
                                  sphere (opTransf (p, m), .7)),
                     opIntersect (box (opTransf (p, m1), vec3 (.55), .1),
                                  sphere (opTransf (p, m1), .7)),
                     .6);
        d = opUnion (d, d3);

        // T1000-ish ball-box magic
        m = trans (vec3 (1.05, -1.3, 2.2 + sin (t)));
        m1 = trans (vec3 (-.05, -.5, 1.5));
        d2 = opBlend (box (opTransf (p, m), vec3 (.7), .1),
                      sphere (opTransf (p, m1), .7),
                      .5 + .25 * cos (iTime));
        d = opUnion (d, d2);

        // dice with holes
        m = trans (vec3 (-1.25, -1.4, -1.25));
        m1 = rotX (90.) * trans (vec3 (-.9, -.9, 1.5));
        mat4 m2 = trans (vec3 (-.9, -1., -.75));
		d2 = opIntersect (box (opTransf (p, m), vec3 (.55), .0),
						  sphere (opTransf (p, m), .7));
		d3 = opSubtract (cylinder (opTransf (p, m1), vec3 (.35, .5, .35)),
					d2);
        d2 = opSubtract (cylinder (opTransf (p, m2), vec3 (.35, .5, .35)),
						 d3);
        m1 = rotZ (90.) * trans (vec3 (.0, -1.75, -.75));
        d3 = opSubtract (cylinder (opTransf (p, m1), vec3 (.35, .5, .35)), d2);
        d = opUnion (d, d3);

		return d;
	}

	float march (vec3 ro, vec3 rd, float maxSteps, float pixelSize)
	{
		bool forceHit = true;
		float infinity = 100000.0;
		float t_min = .00001;
		float t_max = 100.0;
		float t = t_min;
		float candidate = t_min;
		float candidate_error = infinity;
		float w = 1.6;
		float lastd = .0;
		float stepSize = .0;
		float sign = map (ro) < .0 ? -1. : 1.;

	    for (float i = .0; i < maxSteps; i += 1.)
	    {
			float signedd = sign * map (ro + rd * t);
			float d = abs (signedd);

			bool fail = w > 1. && (d + lastd) < stepSize;
			if (fail) {
				stepSize -= w * stepSize;
				w = 1.;
			} else {
				stepSize = signedd * w;
			}

	        lastd = d;
	        float error = d / t;

	        if (!fail && error < candidate_error) {
	        	candidate_error = error;
	        	candidate = t;
	        }

	        if (!fail && error < pixelSize || t > t_max) {
	        	break;
	        }

	        t += stepSize;
	    }

	    if ((t > t_max || candidate_error > pixelSize) && !forceHit) {
	    	return infinity;
	    }

	    return candidate;
	}

	vec3 objectNormal (vec3 p)
	{
	    vec3 e = vec3 (.00001, .0, .0);
	    vec3 n = vec3 (map (p + e.xyy) - map (p - e.xyy),
					   map (p + e.yxy) - map (p - e.yxy),
					   map (p + e.yyx) - map (p - e.yyx));

	    return normalize (n);
	}

	float smoothShadow (vec3 ro, vec3 rd, float tmin, float tmax, float k)
	{
		float res = 1.;
		vec3 p = vec3 (.0);

		for (float t = tmin; t < tmax;) {
			p = ro + rd * t;
			float d = map (p);

			if (d < EPSILON) {
				return .0;
			}

			res = min (res, k * d / t);
			t += d;
		}

		return res;
	}

	float hardShadow (vec3 ro, vec3 rd, float tmin, float tmax)
	{
		for (float t = tmin; t < tmax;) {
			float d = map (ro + rd * t);
			if (d < EPSILON) {
				return .3;
			}

			t += d;
		}

		return 1.;
	}

	vec4 shade (vec3 p)
	{
		float t = .3 * iTime;
		vec3 lPos1 = vec3 (6. * cos (t), -2., -1. + 6. * sin (t));
		vec3 lPos2 = vec3 (3. * cos (-t), -1.5, -1. + 3. * sin (-t));
		vec4 lCol1 = vec4 (.95, .95, .75, 1.);
		vec4 lCol2 = vec4 (.5, .5, .75, .1);
		float reflectance = 1.1;
	    vec3 normal = objectNormal (p);
	    vec3 lDir1 = normalize (lPos1 - p);
	    vec3 lDir2 = normalize (lPos2 - p);
	    vec4 lInt1 = lCol1 * dot (normal, lDir1);
	    vec4 lInt2 = lCol2 * dot (normal, lDir2);

		//vec4 col = smoothShadow (p, normalize (lPos1 - p), .001, 5., 32.) * lInt1;
		//col += smoothShadow (p, normalize (lPos2 - p), .001, 5., 32.) * lInt2;
		vec4 col = hardShadow (p, normalize (lPos1 - p), .001, 10.) * lInt1;
		col += hardShadow (p, normalize (lPos2 - p), .001, 10.) * lInt2;

		return col;
	}

	mat3 camera (vec3 ro, vec3 ta, float cr)
	{
		vec3 cw = normalize (ta - ro);
		vec3 cp = vec3 (sin (cr), cos (cr), .0);
		vec3 cu = normalize (cross (cw, cp));
		vec3 cv = normalize (cross (cu, cw));
	    return mat3 (cu, cv, cw);
	}

	void main()
	{
		float tm = iTime;
		vec2 mouse = iMouse.xy / iResolution.xy;
	    vec3 eye = vec3 (.5 + 5. * cos (.1 * tm + 6. * mouse.x),
						 -4.0 + 4. * mouse.y,
						 .5 + 5. * sin (.1 * tm + 6. * mouse.x));
	    vec3 aim = vec3 (.0, .0, .0);
	    mat3 ca = camera (eye, aim, .0);

	    float u = (fragCoord.x * 2. - 1.) * iResolution.z;
	    float v = fragCoord.y * 2. - 1.;
		vec3 ro = eye;
		vec3 rd = normalize (ca * vec3 (u, -v, 2.));

		vec4 bg = vec4 (.0, .0, .0, 1.);
		vec4 color = bg;
	    const float maxSteps = 64.;
		float pixelSize = 1. / (2. * iResolution.x * iResolution.z);

		// relaxation sphere-tracing, no anti-aliasing
		float t = march (ro, rd, maxSteps, pixelSize);
		//float fac = float (steps) / float (MAX_STEPS);
		//color = mix (gradient (fac), bg, fac);
		color = mix (shade (ro + rd * t), bg, length (rd * t) / 100.);
		/*color = mix (vec4 (-objectNormal (ro + rd * t), 1.),
					 bg, length (rd * t) / 50.);*/

		fragColor = color;
	}
);

const char fragFractal[] = GLSL(
	// the uniform inputs taken over from shadertoy.com
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

	const float PI = 3.14159265358979323846;
	const vec4 red = vec4 (1.0, 0.0, 0.0, 1.0);
	const vec4 green = vec4 (0.0, 1.0, 0.0, 1.0);
	const vec4 blue = vec4 (0.0, 0.0, 1.0, 1.0);
	const vec4 white = vec4 (1.0);
	const vec4 orange = vec4 (1.0, 0.4, 0.125, 1.0);
	const vec4 black = vec4 (0.0, 0.0, 0.0, 1.0);
	const vec4 cyan = vec4 (0.0, 1.0, 1.0, 1.0);
	const vec4 magenta = vec4 (1.0, 0.0, 1.0, 1.0);
	const vec4 yellow = vec4 (1.0, 1.0, 0.0, 1.0);

	float deg2rad (in float degree)
	{
		return degree * PI / 180.0;
	}

	mat2 rot2dZ (in float angle)
	{
		float rad = deg2rad (angle);
		float c = cos (rad);
		float s = sin (rad);

		return mat2 (c, s, -s, c);
	}

	vec4 gradient (float v) {
		float steps = 7.;
		float step = 1. / steps;
		vec4 col = black;

		if (v >= 0.0 && v < step) {
			col = mix (yellow, orange, v * steps);
		} else if (v >= step && v < 2.0 * step) {
			col = mix (orange, red, (v - step) * steps);
		} else if (v >= 2.0 * step && v < 3.0 * step) {
			col = mix (red, magenta, (v - 2.0 * step) * steps);
		} else if (v >= 3.0 * step && v < 4.0 * step) {
			col = mix (magenta, cyan, (v - 3.0 * step) * steps);
		} else if (v >= 4.0 * step && v < 5.0 * step) {
			col = mix (cyan, blue, (v - 4.0 * step) * steps);
		} else if (v >= 5.0 * step && v < 6.0 * step) {
			col = mix (blue, green, (v - 5.0 * step) * steps);
		}

		return col;
	}

	vec4 calc (vec2 c)
	{
    	vec4 col = white;
    	vec2 z = vec2 (0.0);
    	float lengthLimit = 5.0;
    	float maxIter = 255.0;
    	float iter = 0.0;

    	for (float i = .0; i < maxIter; i += 1.) {
    		z = mat2 (z, -z.y, z.x) * z + c;
    		if (length (z) > lengthLimit && iter == 0.0) {
    			iter = i;
    		}
    	}

    	return length (z) <= lengthLimit ? black : gradient (iter / maxIter);
	}

    vec4 mandel (vec2 c, vec2 size)
    {
    	return (calc (c + size * vec2 (-.5, -.5)) +
    			calc (c + size * vec2 ( .5, -.5)) +
    			calc (c + size * vec2 ( .5,  .5)) +
    			calc (c + size * vec2 (-.5,  .5))) / 4.;
    }

	void main ()
	{
		float t = iTime;
		vec2 p = (3.0 * fragCoord.st - vec2 (1.5)) * vec2 (iResolution.z, 1.0);
		float s = 1.0 - sin (t);
		mat2 r = rot2dZ (45.0 * cos (t));
		fragColor = mandel (r * s * p - vec2 (1.501975, 0.0), s * vec2 (1.0/iResolution.x, 1.0/iResolution.y) * vec2 (iResolution.z, 1.0));
    }
);

const char fragLineBox[] = GLSL(
	// the uniform inputs taken over from shadertoy.com
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

	const float PI = 3.14159265358979323846;
	const vec4 red = vec4 (1.0, 0.0, 0.0, 1.0);
	const vec4 green = vec4 (0.0, 1.0, 0.0, 1.0);
	const vec4 blue = vec4 (0.0, 0.0, 1.0, 1.0);
	const vec4 white = vec4 (1.0);
	const vec4 orange = vec4 (1.0, 0.4, 0.125, 1.0);
	const vec4 black = vec4 (0.2, 0.3, 0.2, 1.0);
	const vec4 cyan = vec4 (0.0, 1.0, 1.0, 1.0);
	const vec4 magenta = vec4 (1.0, 0.0, 1.0, 1.0);
	const vec4 yellow = vec4 (1.0, 1.0, 0.0, 1.0);

	float deg2rad (in float degree)
	{
		return degree * PI / 180.0;
	}

	mat2 scale (in vec2 size)
	{
		mat2 mat = mat2 (vec2 (size.x,    0.0),
						 vec2 (   0.0, size.y));

		return mat;
	}

	mat4 trans (in vec3 t)
	{
		mat4 mat = mat4 (vec4 (1.0, 0.0, 0.0, 0.0),
						 vec4 (0.0, 1.0, 0.0, 0.0),
						 vec4 (0.0, 0.0, 1.0, 0.0),
						 vec4 (t.x, t.y, t.z, 1.0));
		return mat;
	}

	mat2 rot2dZ (in float angle)
	{
		float rad = deg2rad (angle);
		float c = cos (rad);
		float s = sin (rad);

		mat2 mat = mat2 (vec2 ( c, s), vec2 (-s, c));

		return mat;
	}

	mat4 rotX (in float angle)
	{
	    float rad = deg2rad (angle);
	    float c = cos (rad);
	    float s = sin (rad);

	    mat4 mat = mat4 (vec4 (1.0, 0.0, 0.0, 0.0),
	                     vec4 (0.0,   c,   s, 0.0),
	                     vec4 (0.0,  -s,   c, 0.0),
	                     vec4 (0.0, 0.0, 0.0, 1.0));

	    return mat;
	}

	mat4 rotY (in float angle)
	{
	    float rad = deg2rad (angle);
	    float c = cos (rad);
	    float s = sin (rad);

	    mat4 mat = mat4 (vec4 (  c, 0.0,  -s, 0.0),
	                     vec4 (0.0, 1.0, 0.0, 0.0),
	                     vec4 (  s, 0.0,   c, 0.0),
	                     vec4 (0.0, 0.0, 0.0, 1.0));

	    return mat;
	}

	mat4 rotZ (in float angle)
	{
	    float rad = deg2rad (angle);
	    float c = cos (rad);
	    float s = sin (rad);

	    mat4 mat = mat4 (vec4 (  c,   s, 0.0, 0.0),
	                     vec4 ( -s,   c, 0.0, 0.0),
	                     vec4 (0.0, 0.0, 1.0, 0.0),
	                     vec4 (0.0, 0.0, 0.0, 1.0));

	    return mat;
	}

	vec4 disc (in vec2 p, in vec2 m, in float r, in vec4 c, in float thickness)
	{
		float d = length (p - m);
		if (d < r) {
			return c * (1.0 - smoothstep (r - thickness, r, d));
		} else {
			return vec4 (0.0, 0.0, 0.0, 1.0);
		}
	}

	vec4 line (in vec2 p, in vec2 a, in vec2 b, in vec4 c, in float thickness)
	{
		vec2 pa = -p - a;
		vec2 ba = b - a;
		float h = clamp (dot (pa, ba) / dot (ba, ba), 0.0, 1.0);
		float d = length (pa - ba * h);

		return c * clamp (((1.0 - d) - (1.0 - thickness)) * 100.0, 0.0, 1.0);
	}

	vec3 gradientColor (float v)
	{
		vec3 color = vec3 (0.0);
	    const vec3 white = vec3 (1.0);
	    const vec3 low = vec3 (0.35, 0.3, 0.4);
	    const vec3 high = vec3 (0.5, 0.6, 0.55);

	    if (v < 0.333) {
	        color = v * low / 0.333;
	    } else if (v < 0.666) {
	        color = (v - 0.333) * (high - low) / 0.333 + low;
	    } else {
	        color = (v - 0.666) * (white - high) / 0.333 + high;
	    }

	    return color;
	}

	struct boxType {vec4 p[8];};

    void main()
    {
		boxType box;
		box.p[0] = vec4 ( 0.1,  0.1,  0.1, 1.0);
		box.p[1] = vec4 ( 0.1, -0.1,  0.1, 1.0);
		box.p[2] = vec4 (-0.1, -0.1,  0.1, 1.0);
		box.p[3] = vec4 (-0.1,  0.1,  0.1, 1.0);
		box.p[4] = vec4 ( 0.1,  0.1, -0.1, 1.0);
		box.p[5] = vec4 ( 0.1, -0.1, -0.1, 1.0);
		box.p[6] = vec4 (-0.1, -0.1, -0.1, 1.0);
		box.p[7] = vec4 (-0.1,  0.1, -0.1, 1.0);

		float thickness = 0.002;

	    float t = 0.4 * PI * iTime;
	    vec2 uvraw = fragCoord.st;// / iResolution.xy;
	    vec2 uv = uvraw - vec2 (0.5, 0.5);
	    uv *= vec2 (iResolution.z, 1.0);

	    float u = uvraw.s * 2.0 * PI;
	    float v = uvraw.t * 2.0 * PI;
	    float uf = (cos (1.1 * u + 7.0 * t) +
					cos (2.3 * v * cos (1.0 * t)) +
					cos (0.3 * u * cos (3.0 * t))) / 3.0;
	    float vf = (cos (2.3 * v + 8.0 * t) +
					cos (1.3 * u * cos (3.0 * t)) +
					cos (1.7 * v * cos (2.0 * t))) / 3.0;
	    float fetcher = (uf * vf + 1.0) / 2.0;
	    vec4 bgCol = vec4 (gradientColor (fetcher), 1.0);

		vec4 disc0Col = disc (uv,
							  vec2 (0.1 + 0.6 * cos (t), 0.2 * sin (t)),
							  0.1,
							  cyan,
							  thickness);
		vec4 disc1Col = disc (uv,
							  vec2 (0.4 * cos (2.0 / 3.0 * t), -0.2 * sin (t)),
							  0.12,
							  magenta,
							  thickness);
		vec4 disc2Col = disc (uv,
							  vec2 (0.2 + 0.3 * cos (4.0 / 5.0 * t),
							  -0.3 * sin (7.0 / 8.0 * t)),
							  0.15,
							  yellow,
							  thickness);

		mat2 rot = rot2dZ (25.0 * iTime);
		mat4 rot3d = rotX (-4.0 * t) * rotY (3.0 * t) * rotZ (2.0 * t);
		mat4 model = trans (vec3 (0.0, 0.0, -0.4)) * rot3d;
		box.p[0] = model * box.p[0];
		box.p[1] = model * box.p[1];
		box.p[2] = model * box.p[2];
		box.p[3] = model * box.p[3];
		box.p[4] = model * box.p[4];
		box.p[5] = model * box.p[5];
		box.p[6] = model * box.p[6];
		box.p[7] = model * box.p[7];

		vec4 boxCol = line (uv,
							box.p[0].xy / box.p[0].z,
							box.p[1].xy / box.p[1].z,
							white,
							thickness);
		boxCol += line (uv,
						box.p[1].xy / box.p[1].z,
						box.p[2].xy / box.p[2].z,
						white,
						thickness);
		boxCol += line (uv,
						box.p[2].xy / box.p[2].z,
						box.p[3].xy / box.p[3].z,
						white,
						thickness);
		boxCol += line (uv,
						box.p[3].xy / box.p[3].z,
						box.p[0].xy / box.p[0].z,
						white,
						thickness);
		boxCol += line (uv,
						box.p[4].xy / box.p[4].z,
						box.p[5].xy / box.p[5].z,
						white,
						thickness);
		boxCol += line (uv,
						box.p[5].xy / box.p[5].z,
						box.p[6].xy / box.p[6].z,
						white,
						thickness);
		boxCol += line (uv,
						box.p[6].xy / box.p[6].z,
						box.p[7].xy / box.p[7].z,
						white,
						thickness);
		boxCol += line (uv,
						box.p[7].xy / box.p[7].z,
						box.p[4].xy / box.p[4].z,
						white,
						thickness);
		boxCol += line (uv,
						box.p[0].xy / box.p[0].z,
						box.p[4].xy / box.p[4].z,
						white,
						thickness);
		boxCol += line (uv,
						box.p[1].xy / box.p[1].z,
						box.p[5].xy / box.p[5].z,
						white,
						thickness);
		boxCol += line (uv,
						box.p[2].xy / box.p[2].z,
						box.p[6].xy / box.p[6].z,
						white,
						thickness);
		boxCol += line (uv,
						box.p[3].xy / box.p[3].z,
						box.p[7].xy / box.p[7].z,
						white,
						thickness);

        fragColor = white - boxCol - disc0Col - disc1Col - disc2Col;
    }
);

#endif // _SHADERS_H
