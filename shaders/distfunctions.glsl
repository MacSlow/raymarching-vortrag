////////////////////////////////////////////////////////////////////////////////
//
// This is an adapted variation of...
//
//  * http://www.iquilezles.org/www/articles/distfunctions/distfunctions.htm
// 
// ... by Íñigo 'iq' Quílez. It's here merely for convenience thus I don't have
// to hop on the web should I want to use one of the more exotic distance
// functions which I don't know by heart.
//
////////////////////////////////////////////////////////////////////////////////

float sdSphere (in vec3 p, in float s)
{
	return length (p) - s;
}

float udBox (in vec3 p, in vec3 b)
{
	return length (max (abs (p) - b, .0));
}

float udRoundBox (in vec3 p, in vec3 b, in float r)
{
	return length (max (abs (p) - b, .0)) - r;
}

float sdBox (in vec3 p, in vec3 b)
{
	vec3 d = abs (p) - b;
	return min (max (d.x, max (d.y, d.z)), .0) + length (max (d, .0));
}

float sdTorus (in vec3 p, in vec2 t)
{
	vec2 q = vec2 (length (p.xz) - t.x, p.y);
	return length (q) - t.y;
}

float sdCylinder (in vec3 p, in vec3 c)
{
	return length (p.xz - c.xy) - c.z;
}

float sdCone (in vec3 p, in vec2 c)
{
    // c must be normalized
    float q = length (p.xy);
    return dot (c, vec2 (q, p.z));
}

float sdPlane (in vec3 p, in vec4 n)
{
	// n must be normalized
	return dot (p, n.xyz) + n.w;
}

float sdHexPrism (in vec3 p, in vec2 h)
{
	vec3 q = abs (p);
	return max (q.z - h.y, max ((q.x * 0.866025 + q.y * .5), q.y) - h.x);
}

float sdTriPrism (in vec3 p, in vec2 h)
{
	vec3 q = abs (p);
	return max (q.z - h.y, max (q.x * 0.866025 + p.y * .5, -p.y) - h.x * .5);
}

float sdCapsule (in vec3 p, in vec3 a, in vec3 b, in float r)
{
	vec3 pa = p - a;
	vec3 ba = b - a;
	float h = clamp (dot (pa, ba) / dot (ba, ba), .0, 1.);
	return length (pa - ba*h) - r;
}

float sdCappedCylinder (in vec3 p, in vec2 h)
{
	vec2 d = abs (vec2 (length (p.xz), p.y)) - h;
	return min (max (d.x, d.y), .0) + length (max (d, .0));
}

float sdCappedCone( in vec3 p, in vec3 c )
{
    vec2 q = vec2 (length (p.xz), p.y);
    vec2 v = vec2 (c.z*c.y/c.x, -c.z);
    vec2 w = v - q;
    vec2 vv = vec2 (dot (v,v), v.x*v.x);
    vec2 qv = vec2 (dot (v,w), v.x*w.x);
    vec2 d = max (qv, .0)*qv/vv;
    return sqrt (dot (w,w) - max (d.x, d.y)) * sign (max (q.y*v.x-q.x*v.y,w.y));
}

float sdEllipsoid (in vec3 p, in vec3 r)
{
    return (length (p/r) - 1.) * min (min (r.x, r.y), r.z);
}

float dot2 (in vec3 v)
{
	return dot (v,v);
}

float udTriangle (in vec3 p, in vec3 a, in vec3 b, in vec3 c)
{
    vec3 ba = b - a;
    vec3 pa = p - a;
    vec3 cb = c - b;
    vec3 pb = p - b;
    vec3 ac = a - c;
    vec3 pc = p - c;
    vec3 nor = cross (ba, ac);

    return sqrt(
    (sign (dot (cross (ba, nor), pa)) +
     sign (dot (cross (cb, nor), pb)) +
     sign (dot (cross (ac, nor), pc)) < 2.0)
     ?
     min (min(
     dot2 (ba*clamp (dot (ba, pa)/dot2 (ba), .0, 1.) - pa),
     dot2 (cb*clamp (dot (cb, pb)/dot2 (cb), .0, 1.) - pb)),
     dot2 (ac*clamp (dot (ac, pc)/dot2 (ac), .0, 1.) - pc))
     :
     dot (nor, pa)*dot (nor, pa)/dot2 (nor));
}

float udQuad (in vec3 p, in vec3 a, in vec3 b, in vec3 c, in vec3 d)
{
    vec3 ba = b - a;
    vec3 pa = p - a;
    vec3 cb = c - b;
    vec3 pb = p - b;
    vec3 dc = d - c;
    vec3 pc = p - c;
    vec3 ad = a - d;
    vec3 pd = p - d;
    vec3 nor = cross (ba, ad);

    return sqrt(
    (sign (dot (cross (ba, nor), pa)) +
     sign (dot (cross (cb, nor), pb)) +
     sign (dot (cross (dc, nor), pc)) +
     sign (dot (cross (ad, nor), pd)) < 3.0)
     ?
     min (min (min(
     dot2 (ba*clamp (dot (ba, pa)/dot2 (ba), .0, 1.)-pa),
     dot2 (cb*clamp (dot (cb, pb)/dot2 (cb), .0, 1.)-pb)),
     dot2 (dc*clamp (dot (dc, pc)/dot2 (dc), .0, 1.)-pc)),
     dot2 (ad*clamp (dot (ad, pd)/dot2 (ad), .0, 1.)-pd))
     :
     dot (nor, pa)*dot (nor, pa)/dot2(nor));
}

float sdTorus82 (in vec3 p, in vec2 t)
{
	vec2 q = vec2 (length2 (p.xz) - t.x,p.y);
	return length8 (q) - t.y;
}

float sdTorus88 (in vec3 p, in vec2 t)
{
	vec2 q = vec2 (length8 (p.xz) - t.x, p.y);
	return length8 (q) - t.y;
}

// -- boolean operators --------------------------------------------------------
float opU (in float d1, in float d2)
{
    return min (d1, d2);
}

float opS (in float d1, in float d2)
{
    return max(-d1, d2);
}

float opI (in float d1, in float d2)
{
    return max (d1, d2);
}

// -- domain operators ---------------------------------------------------------
float opRep (in vec3 p, in vec3 c)
{
    vec3 q = mod(p,c)-0.5*c;
    return primitve( q );
}

vec3 opTx (in vec3 p, in mat4 m)
{
    vec3 q = invert (m)*p;
    return primitive (q);
}

float opScale(in vec3 p, in float s)
{
    return primitive (p/s)*s;
}

// -- distance deformations ----------------------------------------------------
float opDisplace (in vec3 p)
{
    float d1 = primitive (p);
    float d2 = displacement (p);
    return d1 + d2;
}

float opBlend (in vec3 p)
{
    float d1 = primitiveA (p);
    float d2 = primitiveB (p);
    return smin (d1, d2);
}

// -- domain deformations ------------------------------------------------------
float opTwist (in vec3 p)
{
    float c = cos (20.*p.y);
    float s = sin (20.*p.y);
    mat2  m = mat2 (c, -s, s, c);
    vec3  q = vec3 (m*p.xz, p.y);
    return primitive (q);
}

float opCheapBend (in vec3 p)
{
    float c = cos (20.*p.y);
    float s = sin (20.*p.y);
    mat2  m = mat2 (c, -s, s, c);
    vec3  q = vec3 (m*p.xy, p.z);
    return primitive (q);
}
