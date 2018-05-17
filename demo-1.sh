#!/bin/sh

cd ~/src/raymarching-vortrag
build/raymarcher-nongl &
subl ~/src/raymarching-vortrag/include/vec3.h ~/src/raymarching-vortrag/src/raymarchtoolbox.cpp &
