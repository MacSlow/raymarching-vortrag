#!/bin/sh

cd ~/src/raymarching-vortrag
build/raymarcher-gl shaders/boolean-ops.glsl &
subl ~/src/raymarching-vortrag/shaders/boolean-ops.glsl &
