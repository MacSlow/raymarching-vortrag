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
    vec3 position; // self-explanatory
	vec3 ambient;  // kind of a hack
    vec3 diffuse;  // the light's color
    vec3 specular; // kind of a hack
    float attenuation; // attenuation factor
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
mat2 r2d (in float a) { float c = cos(a); float s = sin (a); return mat2 (vec2 (c, s), vec2 (-s, c));}
const float PI = 3.14159265359;
const float INFINITY = 1000000000.;
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

Result maxResult (in Result a, in Result b) {
    if (!a.hit)
        return b;

    if (!b.hit)
        return a;

    bool favourA = a.dist >= b.dist;
    return  Result (true,
                    favourA ? a.point : b.point,
                    favourA ? a.normal : b.normal,
                    favourA ? a.dist : b.dist,
                    favourA ? a.id : b.id);
}

Result sphereIntersect (in Ray ray, in vec3 p, in float r, in int id) {
    Result res = nullResult;

    float a = dot (ray.rd, ray.rd);

    // exit early, if denominator would almost be zero 
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

Result lensIntersect (in Ray ray, in vec3 p, in float r, in int id) {
    Result res = nullResult;
    float s = .8; // normalized value
    vec3 offset = vec3 (r * s, .0, .0);
    Result sphere1 = sphereIntersect (ray,
                                      p - offset, // center of sphere
                                      r,          // radius of sphere
                                      id);        // material-id
    Result sphere2 = sphereIntersect (ray,
                                      p + offset, // center of sphere
                                      r,          // radius of sphere
                                      id);        // material-id

    if (sphere1.hit && sphere2.hit) {
	    Result candidate = maxResult (sphere1, sphere2);
        if (length (candidate.point - p) <= sqrt (r*r*(1. - s*s))) {
            res = candidate;
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

Result boxIntersect (in Ray ray, in vec3 p, in vec3 size, in int id) {
    vec3 min = size * -.5;
    vec3 max = size * .5;
    Result res = nullResult;
    vec3 normal = vec3 (1., .0, .0);

    vec3 d = 1. / ray.rd;
    float txmin = (min.x - ray.ro.x + p.x) * d.x;
    float txmax = (max.x - ray.ro.x + p.x) * d.x;
    float tymin = (min.y - ray.ro.y + p.y) * d.y;
    float tymax = (max.y - ray.ro.y + p.y) * d.y;
    float tzmin = (min.z - ray.ro.z + p.z) * d.z;
    float tzmax = (max.z - ray.ro.z + p.z) * d.z;

    float tXenter = txmin < txmax ? txmin : txmax;
    normal = txmin < txmax ? vec3 (-1., .0, .0) : vec3 (1., .0, .0);
    float tXexit = txmax > txmin ? txmax : txmin;
    float tYenter = tymin < tymax ? tymin : tymax;
    float tYexit = tymax > tymin ? tymax : tymin;
    float tZenter = tzmin < tzmax ? tzmin : tzmax;
    float tZexit = tzmax > tzmin ? tzmax : tzmin;

    float tEnter = tXenter > tYenter ? tXenter : tYenter;
    tEnter = tZenter > tEnter ? tZenter : tEnter;
    float tExit = tXexit < tYexit ? tXexit : tYexit;
    tExit = tExit < tZexit ? tExit : tZexit;

    if (tExit < tEnter) {
        return res;
    }

    return Result (true,
                   ray.ro + tEnter * ray.rd,
                   normal,
                   tEnter,
                   id);
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

    Result lens1 = lensIntersect (ray, vec3 (.0, -1. * (cos (iTime) * .5 + .5), .0), 1.2, 3);

	Result ball1 = sphereIntersect (ray, vec3 (1.0, -1.6, .0), .4, 3);

	Result box1 = boxIntersect (ray, vec3 (-2.0, -1.65, .0), vec3 (.75), 1);

    Result res = minResult (plane1, plane2);
	res = minResult (plane3, res);
	res = minResult (plane4, res);
	res = minResult (plane5, res);
	res = minResult (plane6, res);

    res = minResult (box1, res);

	res = minResult (ball1, res);

	res = minResult (lens1, res);

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
                                              .05,               // reflection strength
                                    		  false,             // does refract
                                    		  .31),              // ior

                                    Material (vec3 (.5),         // PBR: albedo
                                              .0,                // PBR: metallic
                                              1.,                // PBR: roughness
                                              vec3 (.1),         // Blinn/Phong: ambient
                                              vec3 (.9, .3, .6), // Blinn/Phong: diffuse
                                              vec3 (1.),         // Blinn/Phong: specular
                                              20.,               // Blinn/Phong: shininess
                                              vec3 (.0),         // Blinn/Phong: emissive
                                    		  true,              // does reflect
                                              .05,               // reflection strength
                                    		  false,             // does refract
                                    		  .31),              // ior

                                    Material (vec3 (.5),         // PBR: albedo
                                              .0,                // PBR: metallic
                                              1.,                // PBR: roughness
                                              vec3 (.1),         // Blinn/Phong: ambient
                                              vec3 (.6, .9, .3), // Blinn/Phong: diffuse
                                              vec3 (1.),         // Blinn/Phong: specular
                                              20.,               // Blinn/Phong: shininess
                                              vec3 (.0),         // Blinn/Phong: emissive
                                    		  true,              // does reflect
                                              .05,               // reflection strength
                                    		  false,             // does refract
                                    		  .31),              // ior

                                    Material (vec3 (.5),           // PBR: albedo
                                              .0,                  // PBR: metallic
                                              1.,                  // PBR: roughness
                                              vec3 (.1),           // Blinn/Phong: ambient
                                              vec3 (.95, .9, .85), // Blinn/Phong: diffuse
                                              vec3 (.1),           // Blinn/Phong: specular
                                              20.,                 // Blinn/Phong: shininess
                                              vec3 (.0),           // Blinn/Phong: emissive);
                                    		  true,                // does reflect
                                              .1,                 // reflection strength
                                    		  true,                // does refract
                                    		  .85));               // ior

vec3 shade (in Ray ray, in Result res) {
    vec3 amb = vec3 (.0);
    vec3 diffC[4];

	float pattern1 = saturate (pow (abs(15. * cos(res.point.x+iTime) * sin (res.point.z+iTime)), .3));
    float pattern2 = saturate (pow (abs(2. * cos(res.point.x+iTime) * sin (res.point.z+iTime) * .5 + .5), .3));
    float pattern3 = saturate (pow (length (4.*sin(mod((res.point.y*res.point.x), .3))), .125));
    float pattern4 = saturate (mod (length(res.point*sin(.1*iTime)), .5));

    diffC[0] = mix (vec3 (.3, .6, .9), vec3 (.8), 1. - pattern1);
    diffC[1] = mix (vec3 (.9, .3, .6), vec3 (.9), 1. - pattern2);
    diffC[2] = mix (vec3 (.6, .9, .3), vec3 (.5), 1. - pattern4);
    diffC[3] = vec3 (.0);
    vec3 specC = vec3 (.0);
    vec3 specC2 = vec3 (.0);
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
    float diff2 = clamp (dot (res.normal, lDir2), .0, 1.);
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

void main()
{
    // normalize and aspect-correct UVs
    vec2 uv = fragCoord.xy; // / iResolution.xy;
    uv = uv * 2. - 1.;
    float aspect = iResolution.x / iResolution.y;
    uv.x *= aspect;

    // generate primary ray from camera
    vec3 ro = vec3 (.0, .0, -3.);
    ro.xz *= r2d (PI*sin (.5*PI*(iMouse.x/iResolution.x * 2. - 1.)));
    ro.yz *= r2d (.25*PI*sin (-.5*PI*(iMouse.y/iResolution.y * 2. - 1.)));
    vec3 lookAt = vec3 (.0);
    float zoom = 1.75;
    Ray ray = cameraRay (uv, ro, lookAt, zoom);

    // ray-trace the scene (primary view-ray)
    Result res = trace (ray);
    vec3 col = material[res.id].doesRefract ? vec3 (.0) : shade (ray, res);

    if (material[res.id].doesReflect) {
        // first reflection bounce
	    Ray reflectedRay = Ray (res.point + .01 * res.normal,
                                normalize (reflect (ray.rd, res.normal)));
        Result bounce = trace (reflectedRay);
        col += material[res.id].reflAmount * shade (reflectedRay, bounce);

        // second reflection bounce
        reflectedRay = Ray (bounce.point + .01 * bounce.normal,
                            normalize (reflect (reflectedRay.rd, bounce.normal)));
        bounce = trace (reflectedRay);
        col +=  material[res.id].reflAmount * shade (reflectedRay, bounce);
    }

    if (material[res.id].doesRefract) {
        // first refraction
	    Ray refractedRay = Ray (res.point - .01 * res.normal,
                                normalize (refract (ray.rd,
                                                    res.normal,
                                                    material[res.id].ior)));
        Result refracted = trace (refractedRay);
        col += .25 * shade (refractedRay, refracted);

        // second refraction
	    refractedRay = Ray (refracted.point - .01 * refracted.normal,
                            normalize (refract (refractedRay.rd,
                                                refracted.normal,
                                                material[res.id].ior)));
        refracted = trace (refractedRay);
        col += .25 * shade (refractedRay, refracted);

        // third refraction
	    refractedRay = Ray (refracted.point - .01 * refracted.normal,
                            normalize (refract (refractedRay.rd,
                                                refracted.normal,
                                                material[res.id].ior)));
        refracted = trace (refractedRay);
        col += .25 * shade (refractedRay, refracted);
    }

    // tone-map and gamma-correct
    col = col / (1. + col);
    col = sqrt (col);

	fragColor = vec4 (col, 1.);
}
