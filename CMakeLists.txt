cmake_minimum_required (VERSION 2.8.12)
project (raymarching)

set (CMAKE_CXX_COMPILER "g++")
set (CMAKE_EXPORT_COMPILE_COMMANDS "ON")

include (FindPkgConfig)
pkg_check_modules (SDL2 sdl2 REQUIRED)
include_directories (include ${SDL2_INCLUDE_DIRS})

add_compile_options(-std=c++14 -Werror -Wall -pedantic -fno-omit-frame-pointer -Ofast -march=native)
#add_compile_options(-std=c++14 -Werror -Wall -pedantic -fno-omit-frame-pointer -fsanitize=undefined,leak,address)

add_library (ApplicationLib src/application.cpp)
add_library (RaymarchingLib src/raymarching.cpp src/vec3.cpp)

add_executable (raymarching src/main.cpp)
target_link_libraries (raymarching
	ApplicationLib
	RaymarchingLib
	#-fsanitize=leak,address,undefined
	${SDL2_LDFLAGS}
	-pthread
	-s
)

add_custom_target (valgrind
	DEPENDS raymarching
	COMMAND valgrind --track-origins=yes --show-leak-kinds=all  --leak-check=full -v ./raymarching
)

add_custom_target (perf
	DEPENDS raymarching
	COMMAND perf stat ./raymarching
)

add_custom_target (report
	DEPENDS raymarching
	COMMAND perf record -g ./raymarching
	COMMAND perf report -g 'graph,0.5,caller' --sort comm,dso,sym
)

add_custom_target (tiny
	DEPENDS raymarching
	COMMAND sstrip ./raymarching
	COMMAND gzip -cn9 ./raymarching >small
	COMMAND cat ../data/tiny ./small >compact
	COMMAND chmod +x ./compact
	COMMAND rm ./small ./raymarching
	COMMAND mv ./compact ./raymarching
)

add_custom_target (tidy
	DEPENDS raymarching
	COMMAND clang-tidy -p=. ../src/*.cpp
)
