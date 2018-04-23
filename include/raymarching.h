#ifndef _RAYMARCHING_H
#define _RAYMARCHING_H

#include <SDL.h>
#include <vector>
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
    size_t _width;
    size_t _height;
    std::chrono::time_point<std::chrono::high_resolution_clock> _startTimeStamp;
    SurfaceBuffer _bufferSurface;
};

#endif
