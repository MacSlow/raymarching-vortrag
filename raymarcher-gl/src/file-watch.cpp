#include <fstream>
#include <iostream>
#include <functional>

#include "file-watch.h"

using std::cout;
using std::ifstream;
using std::string;

int get_cheap_checksum_of (std::shared_ptr<std::string> filename)
{
    string line;
    ifstream file (*filename);
    if (!file)
    	return 0;

    unsigned sum = 0;
    while (!getline (file, line).eof ())
        for (auto c : line)
            sum += static_cast<int> (c);

    return sum;
}

bool did_file_change (std::shared_ptr<std::string> filename, int checksum)
{
    return checksum != get_cheap_checksum_of (filename);
}

void update_checksum (FileWatchParameters* file_watch_parameters)
{
	file_watch_parameters->checksum = get_cheap_checksum_of (file_watch_parameters->filename);
}
