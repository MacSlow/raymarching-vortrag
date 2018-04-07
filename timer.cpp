// g++ -Wall -Werror -pedantic -std=c++17 `sdl2-config --cflags --libs`
// -fsanitize=address,leak,undefined timer.cpp -o timer
// LANG=C ./timer shader.frag

#include <SDL.h>
#include <fstream>
#include <iostream>
#include <string>
#include <functional>

using std::cout;
using std::ifstream;
using std::string;

struct FileWatchParameters {
    const string& filename;
    int checksum;
};

int get_cheap_checksum_of (const string& filename)
{
    string line;
    ifstream file (filename);
    if (!file)
    	return 0;

    unsigned sum = 0;
    while (!getline (file, line).eof ())
        for (auto c : line)
            sum += static_cast<int> (c);

    return sum;
}

bool did_file_change (const string& filename, int checksum)
{
    return checksum != get_cheap_checksum_of (filename);
}

void read_shader_from (const string& filename)
{
    string line;
    ifstream file (filename);
    if (!file)
    	return;

    while (!getline (file, line).eof ())
        cout << line << '\n';
}

void update_checksum (FileWatchParameters* file_watch_parameters)
{
	file_watch_parameters->checksum = get_cheap_checksum_of (file_watch_parameters->filename);
}

Uint32 file_watcher (Uint32 interval, void* parameters)
{
    FileWatchParameters* file_watch_parameters =
        reinterpret_cast<FileWatchParameters*> (parameters);

    auto file_changed = did_file_change (file_watch_parameters->filename,
                                         file_watch_parameters->checksum);
    if (file_changed) {
        read_shader_from (file_watch_parameters->filename);
        update_checksum (file_watch_parameters);
    }

    return interval;
}

int main (int argc, char** argv)
{
    if (argc != 2) {
        cout << "Usage: " << argv[0] << " <shader-file>\n";
        return 1;
    }

    SDL_Init (SDL_INIT_VIDEO | SDL_INIT_TIMER);
    SDL_Window* window = SDL_CreateWindow (
        "SDL-Timer Test", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 400,
        400, SDL_WINDOW_RESIZABLE);

	SDL_TimerID timer_id = 0;
	string filename (argv[1]);
	Uint32 delay_in_ms = 1000;
	FileWatchParameters file_watch_parameters{filename, 0};
	update_checksum (&file_watch_parameters);
	auto parameters = reinterpret_cast<void*> (&file_watch_parameters);
	timer_id = SDL_AddTimer (delay_in_ms, file_watcher, parameters);

    bool running = true;

    while (running) {
        SDL_Event event;
        while (SDL_PollEvent (&event)) {
            switch (event.type) {
            case SDL_KEYUP:
                if (event.key.keysym.sym == SDLK_ESCAPE)
                    running = false;
                break;

            case SDL_WINDOWEVENT:
                if (event.window.event == SDL_WINDOWEVENT_CLOSE)
                    running = false;
                break;

            case SDL_QUIT:
                running = false;
                break;
            }
        }
        SDL_Delay (1000 / 60);
    }

    SDL_RemoveTimer (timer_id);
    SDL_DestroyWindow (window);
    SDL_Quit ();

    return 0;
}
