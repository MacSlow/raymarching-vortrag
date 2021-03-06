#ifndef _RAYMARCHING_H
#define _RAYMARCHING_H

#include <SDL.h>
#include <vector>
#include <array>
#include <chrono>

#include "raymarchtoolbox.h"

using SurfaceBuffer = std::vector<Color>;
enum class RGB {R = 0, G, B};

class Raymarching {
  public:
    Raymarching (size_t width, size_t height);

    void update ();
    void updateMT ();
    void paint (const SDL_Window* window) const;

  private:
    size_t _width = 0;
    size_t _height = 0;
    std::chrono::time_point<std::chrono::high_resolution_clock> _startTimeStamp;
    SurfaceBuffer _bufferSurface;
};

#endif
