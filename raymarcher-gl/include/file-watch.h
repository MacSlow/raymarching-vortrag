#ifndef _FILE_WATCH_H
#define _FILE_WATCH_H

#include <SDL.h>
#include <string>
#include <memory>

struct FileWatchParameters {
    std::shared_ptr<std::string> filename;
    int checksum;
};

int get_cheap_checksum_of (std::shared_ptr<std::string> filename);
bool did_file_change (std::shared_ptr<std::string> filename, int checksum);
void update_checksum (FileWatchParameters* file_watch_parameters);

#endif
