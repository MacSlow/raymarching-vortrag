cmake_minimum_required (VERSION 2.8.12)
project (raymarcher-nongl)
project (raymarcher-gl)

set (CMAKE_CXX_COMPILER "clang++")
set (CMAKE_EXPORT_COMPILE_COMMANDS "ON")

include (FindPkgConfig)
pkg_check_modules (SDL2 sdl2 REQUIRED)
include_directories (include ${SDL2_INCLUDE_DIRS})

add_compile_options(-DGL_GLEXT_PROTOTYPES -std=c++17 -Werror -Wall -pedantic -fno-omit-frame-pointer -Ofast -march=native)
#add_compile_options(-DGL_GLEXT_PROTOTYPES 	-std=c++17 -Werror -Wall -pedantic -fno-omit-frame-pointer -fsanitize=undefined,leak,address)

add_library (ApplicationLib src/application.cpp)
add_library (RaymarchingLib src/raymarching.cpp
	                        src/raymarchtoolbox.cpp
	                        src/vec3.cpp)

add_executable (raymarcher-nongl src/main.cpp)
target_link_libraries (raymarcher-nongl
	ApplicationLib
	RaymarchingLib
	#-fsanitize=leak,address,undefined
	${SDL2_LDFLAGS}
	-pthread
	-s
)

include (FindPkgConfig)
pkg_check_modules (SDL2_IMAGE SDL2_image REQUIRED)
include_directories (raymarcher-gl/include ${SDL2_IMAGE_INCLUDE_DIRS})
add_library (DisplayLib raymarcher-gl/src/display.cpp)
add_library (OpenGLLib raymarcher-gl/src/opengl.cpp
	                   raymarcher-gl/src/texture.cpp)
add_library (FileWatchLib raymarcher-gl/src/file-watch.cpp)

add_executable (raymarcher-gl raymarcher-gl/src/main.cpp)
target_link_libraries (raymarcher-gl
	DisplayLib
	OpenGLLib
	FileWatchLib
	GL
	#-fsanitize=leak,address,undefined
	${SDL2_IMAGE_LDFLAGS}
	-s
)

add_custom_target (valgrind
	DEPENDS raymarcher-nongl
	COMMAND valgrind --track-origins=yes --show-leak-kinds=all  --leak-check=full -v ./raymarcher-nongl
)

add_custom_target (perf
	DEPENDS raymarcher-nongl
	COMMAND perf stat ./raymarcher-nongl
)

add_custom_target (report
	DEPENDS raymarcher-nongl
	COMMAND perf record -g ./raymarcher-nongl
	COMMAND perf report -g 'graph,0.5,caller' --sort comm,dso,sym
)

add_custom_target (tiny
	DEPENDS raymarcher-nongl
	COMMAND sstrip ./raymarcher-nongl
	COMMAND gzip -cn9 ./raymarcher-nongl >small
	COMMAND cat ../data/tiny ./small >compact
	COMMAND chmod +x ./compact
	COMMAND rm ./small ./raymarcher-nongl
	COMMAND mv ./compact ./raymarcher-nongl
)

add_custom_target (tidy
	DEPENDS raymarcher-nongl
	COMMAND clang-tidy -p=. ../src/*.cpp
)
