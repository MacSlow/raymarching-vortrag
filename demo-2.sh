#!/bin/sh

cd ~/src/raymarching-vortrag
build/raymarcher-gl shaders/basic-3d-raymarching.glsl &
subl ~/src/raymarching-vortrag/shaders/basic-3d-raymarching.glsl &
