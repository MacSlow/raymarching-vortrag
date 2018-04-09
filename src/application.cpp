#include <iostream>
#include <sstream>
#include <memory>
#include "application.h"

using namespace std;

#define WIN_TITLE "NonGL-Raymarcher by MacSlow"
#define newline '\n'

void Application::initialize ()
{
    if (_initialized)
        return;

    int result = 0;
    SDL_ClearError ();
    result = SDL_Init (SDL_INIT_VIDEO);
    if (result != 0) {
        cout << "SDL_Init() failed: " << SDL_GetError () << newline;
        _initialized = false;
        return;
    }

    _initialized = true;
}

Application::Application (size_t width, size_t height)
    : _initialized (false), _window (NULL), _running (false), _paused (false),
      _seconds (0), _allFrames (0), _min (0), _avg (0), _max (0)
{
    initialize ();

    SDL_ClearError ();
    _window = SDL_CreateWindow (WIN_TITLE, SDL_WINDOWPOS_UNDEFINED,
                                SDL_WINDOWPOS_UNDEFINED, width, height, 0);
    if (!_window) {
        cout << "window creation failed: " << SDL_GetError () << newline;
        return;
    }

    _raymarching = make_unique<Raymarching> (width, height);
}

Application::~Application ()
{
    SDL_DestroyWindow (_window);
    SDL_Quit ();
}

void Application::handle_events ()
{
    SDL_Event event;
    while (SDL_PollEvent (&event)) {
        switch (event.type) {
        case SDL_KEYDOWN:
            if (event.key.keysym.sym == SDLK_ESCAPE)
                _running = false;
            else if (event.key.keysym.sym == SDLK_SPACE) {
                _paused = !_paused;
                update_title ();
            }
            break;

        case SDL_QUIT:
            _running = false;
            break;
        }
    }
}

void Application::run ()
{
    if (!_initialized)
        return;

    _running = true;

    while (_running) {
        handle_events ();
        update ();
    }
}

void Application::update_title ()
{
    if (_paused) {
        stringstream title;
        title << WIN_TITLE << " - paused";
        string str (title.str ());
        SDL_SetWindowTitle (_window, str.c_str ());
    } else {
        stringstream title;
        title << WIN_TITLE << " - " << _min << "/" << _avg << "/" << _max
              << " fps";
        string str (title.str ());
        SDL_SetWindowTitle (_window, str.c_str ());
    }
}

void Application::update_framerate ()
{
    static unsigned int fps = 0;
    static unsigned int lastTick = 0;
    static unsigned int currentTick = 0;

    ++fps;
    currentTick = SDL_GetTicks ();
    if (currentTick - lastTick > 1000) {
        update_title ();

        ++_seconds;
        lastTick = currentTick;

        if (_max < fps)
            _max = fps;

        if (_seconds > 0) {
            _allFrames += fps;
            _avg = _allFrames / _seconds;
        }

        if (_min > fps || _min == 0)
            _min = fps;

        fps = 0;
    }
}

void Application::update ()
{
    if (!_initialized)
        return;

    if (_paused)
        return;

    update_framerate ();

    _raymarching->updateMT ();
    _raymarching->paint (_window);
}
