#ifndef _RAYMARCHING_H
#define _RAYMARCHING_H

#include <SDL.h>
#include <vector>
#include <tuple>
#include <array>
#include <chrono>

using Color = std::array<float, 3>;
using UV = std::array<float, 2>;
using Resolution = std::array<float, 2>;
using SurfaceBuffer = std::vector<Color>;
using Seconds = float;
enum class RGB {R = 0, G, B};
using Vec2 = float[2];
using Vec3 = float[3];

class Raymarching {
  public:
    Raymarching (size_t width, size_t height);

    void reset ();
    void update ();
    void updateMT ();
    void paint (const SDL_Window* window) const;

  private:
    size_t _width = 0;
    size_t _height = 0;
    std::chrono::time_point<std::chrono::high_resolution_clock> _start;
    SurfaceBuffer _bufferSurface;
};

#endif // _RAYMARcHING_H
