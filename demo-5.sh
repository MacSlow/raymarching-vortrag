#!/bin/sh

cd ~/src/raymarching-vortrag
build/raymarcher-gl shaders/texture-terrain.glsl 720 400 60 data/noise-256x256.png &
subl ~/src/raymarching-vortrag/shaders/texture-terrain.glsl &
