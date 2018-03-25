#include "application.h"

#define WIDTH 512
#define HEIGHT 256

int main (int argc, char** argv)
{
    Application app (WIDTH, HEIGHT);
    app.run ();

    return 0;
}
