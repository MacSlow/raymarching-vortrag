#include <future>

#include "raymarching.h"
#include "raymarchtoolbox.h"

using namespace std;
using std::chrono::high_resolution_clock;

#define NUM_CHANNELS 4
#define FULL_INTENSITY 255
#define MAX_THREADS 4

Raymarching::Raymarching (size_t width, size_t height)
    : _width (width), _height (height)
{
    size_t size = _width * _height;
    _bufferSurface = SurfaceBuffer (size);
    _startTimeStamp  = high_resolution_clock::now ();
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

void writeScanline (future<Input>& f)
{
    Input input = f.get ();
    Resolution res {{input.resolution_x, input.resolution_y}};

    for (size_t x = 0; x < input.width; ++x) {
        size_t index = x + input.y * input.width;
        UV uv = {{static_cast<float> (x*input.recip_width),
                  static_cast<float> (input.y*input.recip_height)}};
        std::chrono::duration<float> elapsed = high_resolution_clock::now() -
                                               input.start;
        input.dst[index] = computeColor (uv, elapsed.count (), res);
    }
}

void Raymarching::updateMT ()
{
    Resolution res {{static_cast<float> (_width),
                     static_cast<float> (_height)}};

	float recip_width = 1.f / static_cast<float> (_width);
	float recip_height = 1.f / static_cast<float> (_height);

    for (size_t y = 0; y < _height; y += MAX_THREADS) {
        auto now = high_resolution_clock::now();
        Input input = {_bufferSurface, .0f, .0f, .0f, .0f, .0f, .0f, now};

        promise<Input> p[MAX_THREADS];
        future<Input> fInput[MAX_THREADS];
        future<void> f[MAX_THREADS];

        for (int i= 0; i<MAX_THREADS; ++i) {
            fInput[i] = p[i].get_future();
            f[i] = future<void> (async (launch::async,
                                 writeScanline,
                                 ref (fInput[i])));
            input.dst = _bufferSurface;
            input.width = _width;
            input.y = y+i;
            input.recip_width = recip_width;
            input.recip_height = recip_height;
            input.resolution_x = res[0];
            input.resolution_y = res[1];
            input.start = _startTimeStamp;
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
            std::chrono::duration<float> elapsed = high_resolution_clock::now() -
                                                   _startTimeStamp;
            _bufferSurface[index] = computeColor (uv, elapsed.count (), res);
        }
    }
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
