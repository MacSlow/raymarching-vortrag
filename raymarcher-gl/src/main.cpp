////////////////////////////////////////////////////////////////////////////////
//
// A desktop-based GLSL-environment similar to the WebGL-based site
// www.shadertoy.com
//
// Copyright 2015-2016 Mirco Müller
//
// Author(s):
//   Mirco "MacSlow" Müller <macslow@gmail.com>
//
// This _programram is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License version 3, as published
// by the Free Software Foundation.
//
// This _programram is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranties of
// MERCHANTABILITY, SATISFACTORY QUALITY, or FITNESS FOR A PARTICULAR
// PURPOSE.  See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this _programram.  If not, see <http://www.gnu.org/licenses/>.
//
////////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include "display.h"

using std::cout;

int main (int argc, char* argv[])
{
	if (argc == 1) {
        cout << "Usage: " << argv[0]
             << " <shader-file> [<width> <height> <fps> <texture-file> "
                "<texture-file> <texture-file> <texture-file>]\n";
        return 1;
	}

    Display* display = new Display (
        argc >= 2 ? argv[1] : nullptr, argc >= 3 ? atoi (argv[2]) : 640,
        argc >= 4 ? atoi (argv[3]) : 360, argc >= 5 ? atoi (argv[4]) : 30,
        argc >= 6 ? argv[5] : nullptr, argc >= 7 ? argv[6] : nullptr,
        argc >= 8 ? argv[7] : nullptr, argc >= 9 ? argv[8] : nullptr);
    display->run ();
	delete display;

	return 0;
}
