#include <algorithm>
#include <random>
#include <future>
#include <iostream>
#include "raymarching.h"

using namespace std;
using std::chrono::high_resolution_clock;

#define NUM_CHANNELS 4
#define NO_INTENSITY 0
#define FULL_INTENSITY 255

Raymarching::Raymarching (size_t width, size_t height)
    : _width (width), _height (height)
{
    size_t size = _width * _height;
    _bufferSurface = SurfaceBuffer (size);
    _start  = high_resolution_clock::now ();
}

struct Input
{
    SurfaceBuffer& dst;
    float width;
    float y;
    float recip_width;
    float recip_height;
    float resolution_x;
    float resolution_y;
    chrono::time_point<chrono::high_resolution_clock>& start;
};

Color computeColor (const UV& uv, const Seconds& seconds, const Resolution& res);

void oneLine (future<Input>& f)
{
    Input i = f.get ();
    Resolution res {{i.resolution_x, i.resolution_y}};

    for (size_t x = 0; x < i.width; ++x) {
        size_t index = x + i.y * i.width;
        UV uv = {{static_cast<float> (x*i.recip_width),
                  static_cast<float> (i.y*i.recip_height)}};
        std::chrono::duration<float> elapsed = high_resolution_clock::now() - i.start;
        i.dst[index] = computeColor (uv, elapsed.count (), res);
    }
}

#define MAX_THREADS 4

void Raymarching::updateMT ()
{
    Resolution res {{static_cast<float> (_width),
                     static_cast<float> (_height)}};

	float recip_width = 1.f / static_cast<float> (_width);
	float recip_height = 1.f / static_cast<float> (_height);

    for (size_t y = 0; y < _height; y += MAX_THREADS) {
        auto foo = high_resolution_clock::now();
        Input input = {_bufferSurface, .0f, .0f, .0f, .0f, .0f, .0f, foo};

        promise<Input> p[MAX_THREADS];
        future<Input> fInput[MAX_THREADS];
        future<void> f[MAX_THREADS];

        for (int i= 0; i<MAX_THREADS; ++i) {
            fInput[i] = p[i].get_future();
            f[i] = future<void> (async (launch::async,
                                 oneLine,
                                 ref (fInput[i])));
            input.dst = _bufferSurface;
            input.width = _width;
            input.y = y+i;
            input.recip_width = recip_width;
            input.recip_height = recip_height;
            input.resolution_x = res[0];
            input.resolution_y = res[1];
            input.start = _start;
            p[i].set_value (input);
        }

        for (int i= 0; i<MAX_THREADS; ++i) {
            f[i].get ();
        }
    }
}

void Raymarching::update ()
{
    Resolution res {{static_cast<float> (_width),
                     static_cast<float> (_height)}};

    float recip_width = 1.f / static_cast<float> (_width);
    float recip_height = 1.f / static_cast<float> (_height);

    for (size_t y = 0; y < _height; ++y) {
        for (size_t x = 0; x < _width; ++x) {
            size_t index = x + y * _width;
            UV uv = {{static_cast<float> (x*recip_width),
                      static_cast<float> (y*recip_height)}};
            std::chrono::duration<float> elapsed = high_resolution_clock::now() - _start;
            _bufferSurface[index] = computeColor (uv, elapsed.count (), res);
        }
    }
}

// auto mix = [](const float a, const float b, const float factor){}
// auto mix = [](const vec2& a, const vec2& b, const float factor){}
// auto mix = [](const vec3& a, const vec3& b, const float factor){}

auto length = [](const Vec2& p) {
	return sqrt (p[0]*p[0] + p[1]*p[1]);
};

auto clamp = [](const float v, const float edge0, const float edge1) {
	return (v < edge0) ? edge0 : (v > edge1) ? edge1: v;
};

auto saturate = [](const float v) {
	return clamp (v, .0f, 1.f);
};

auto smoothstep = [](const float v, const float edge0, const float edge1) {
    float t = clamp((v - edge0) / (edge1 - edge0), .0f, 1.f);
    return t * t * (3.f - 2.f * t);
};

Color computeColor (const UV& uv,
                    const Seconds& seconds,
                    const Resolution& res)
{
	Color color = {{uv[0], uv[1], .0f}};
	float aspect = res[0] / res[1];
	Vec2 p = {.3f, -.2};
	Vec2 q = {-.2f, .4f};

	float c = cosf (seconds);
	float s = sinf (seconds);
	p[0] = aspect*((uv[0] + p[0]*c)*2.f - 1.f);
	p[1] = ((uv[1] + p[1]*s)*2.f - 1.f);
	q[0] = aspect*((uv[0] + q[0]*s)*2.f - 1.f);
	q[1] = ((uv[1] + q[1]*c)*2.f - 1.f);

	float r = 0.1f;

	float d = saturate (length (p) - r);
	d = min (d, saturate (length (q) - r));
	d = smoothstep (d, .1f, .11f);

	color [0] = 1.f - d;
	color [1] = 1.f - d;
	color [2] = 1.f - d;

	return color;
}

void Raymarching::paint (const SDL_Window* window) const
{
    size_t pitch = _width * NUM_CHANNELS;
    size_t size = _height * pitch;
    std::vector<unsigned char> buffer;
    buffer.reserve (size);

    size_t indexSurface = 0;
    size_t indexBuffer = 0;
    Color value = {{1.0f, 0.5f, 0.25f}};

    for (size_t y = 0; y < _height; ++y) {
        for (size_t x = 0; x < _width; ++x) {
            indexSurface = x * NUM_CHANNELS + y * pitch;
            indexBuffer = x + y * _width;
            value = _bufferSurface[indexBuffer];
            buffer[indexSurface] = static_cast<unsigned char> (255.0f * value[0]);
            buffer[indexSurface + 1] = static_cast<unsigned char> (255.0f * value[1]);
            buffer[indexSurface + 2] = static_cast<unsigned char> (255.0f * value[2]);
            buffer[indexSurface + 3] = FULL_INTENSITY;
        }
    }

    SDL_Surface* src = nullptr;
    src = SDL_CreateRGBSurfaceWithFormatFrom (
        buffer.data (), _width, _height, 32, pitch, SDL_PIXELFORMAT_RGB888);
    SDL_Surface* dst = SDL_GetWindowSurface (const_cast<SDL_Window*> (window));
    SDL_BlitSurface (src, NULL, dst, NULL);
    SDL_FreeSurface (src);
    SDL_UpdateWindowSurface (const_cast<SDL_Window*> (window));
}
