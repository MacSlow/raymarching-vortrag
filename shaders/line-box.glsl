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
