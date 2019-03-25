cmake_minimum_required (VERSION 3.5.0)

# With System Libraries (on Linux or Windows/MSYS/MinGW):
# cmake ..
#
# With Developer Libs:
# cmake .. -DSDL2_DIR=D:/src/sdl/VC/SDL2-2.0.8 -DSDL2_IMAGE_DIR=D:\src\sdl\VC\SDL2_image-2.0.3
#
# On Windows:
# cmake -G "Visual Studio 15 2017 Win64" ..

project (raymarcher)
set (CMAKE_CXX_COMPILER "g++")
set (CMAKE_EXPORT_COMPILE_COMMANDS "ON")
set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

include (FindPkgConfig)
pkg_check_modules(GLEW REQUIRED glew)
pkg_check_modules(SDL2 REQUIRED sdl2)
pkg_check_modules(SDL2_IMAGE REQUIRED SDL2_image)
include_directories (include
					 ${SDL2_INCLUDE_DIRS}
					 ${SDL2_IMAGE_INCLUDE_DIRS}
					 ${GLEW_INCLUDE_DIRS})
link_directories (${SDL2_LIBRARY_DIRS}
				  ${SDL2_IMAGE_LIBRARY_DIRS}
				  ${GLEW_LIBRARY_DIRS})

add_compile_options(-Werror -Wall -pedantic -std=c++14 -Ofast -march=native -DGL_GLEXT_PROTOTYPES)
#add_compile_options(-Werror -Wall -pedantic -std=c++14 -fsanitize=leak,address,undefined -DGL_GLEXT_PROTOTYPES)

add_library (ApplicationLib src/application.cpp)
add_library (RaymarchingLib src/raymarching.cpp
	                        src/raymarchtoolbox.cpp
	                        src/vec3.cpp)

add_executable (raymarcher-nongl src/main.cpp)

target_link_libraries (raymarcher-nongl
					   ApplicationLib
					   RaymarchingLib
					   #-fsanitize=leak,address,undefined
					   ${SDL2_LIBRARIES}
					   pthread
					   -s)

include_directories (raymarcher-gl/include
					 ${SDL2_INCLUDE_DIRS}
					 ${SDL2_IMAGE_INCLUDE_DIRS}
					 ${GLEW_INCLUDE_DIRS})
add_library (DisplayLib raymarcher-gl/src/display.cpp)
add_library (OpenGLLib raymarcher-gl/src/opengl.cpp
	                   raymarcher-gl/src/texture.cpp)
add_library (FileWatchLib raymarcher-gl/src/file-watch.cpp)

add_executable (raymarcher-gl raymarcher-gl/src/main.cpp)
target_link_libraries (raymarcher-gl
						DisplayLib
						OpenGLLib
						FileWatchLib
						#GL # for Linux
						opengl32 # for Windows/MSYS/MinGW
						#-fsanitize=leak,address,undefined
						${SDL2_LIBRARIES}
						${SDL2_IMAGE_LIBRARIES}
						${GLEW_LIBRARIES} 
						-s)

# only used under Linux
add_custom_target (valgrind
	DEPENDS raymarcher-nongl
	COMMAND valgrind --track-origins=yes --show-leak-kinds=all  --leak-check=full -v ./raymarcher-nongl
)

# only used under Linux
add_custom_target (perf
	DEPENDS raymarcher-nongl
	COMMAND perf stat ./raymarcher-nongl
)

# only used under Linux
add_custom_target (report
	DEPENDS raymarcher-nongl
	COMMAND perf record -g ./raymarcher-nongl
	COMMAND perf report -g 'graph,0.5,caller' --sort comm,dso,sym
)

# only used under Linux
add_custom_target (tiny
	DEPENDS raymarcher-nongl
	COMMAND sstrip raymarcher-nongl
	COMMAND xz -z -9 -e -T0 raymarcher-nongl
	COMMAND cat ../data/tiny raymarcher-nongl.xz >compact
	COMMAND chmod +x compact
	COMMAND rm raymarcher-nongl.xz
	COMMAND mv compact raymarcher-nongl
)

add_custom_target (tiny-gl
	DEPENDS raymarcher-gl
	COMMAND sstrip raymarcher-gl
	COMMAND xz -z -9 -e -T0 raymarcher-gl
	COMMAND cat ../data/tiny-gl raymarcher-gl.xz >compact
	COMMAND chmod +x compact
	COMMAND rm raymarcher-gl.xz
	COMMAND mv compact raymarcher-gl
)

# only used under Linux
add_custom_target (tidy
	DEPENDS raymarcher-nongl
	COMMAND clang-tidy -p=. ../src/*.cpp
)

