#include "application.h"

#define WIDTH 640
#define HEIGHT 360

int main (int argc, char** argv)
{
    Application app (WIDTH, HEIGHT);
    app.run ();

    return 0;
}
