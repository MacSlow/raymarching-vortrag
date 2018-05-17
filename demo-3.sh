#!/bin/sh

cd ~/src/raymarching-vortrag
build/raymarcher-gl shaders/mirror-room.glsl &
subl ~/src/raymarching-vortrag/shaders/mirror-room.glsl &
