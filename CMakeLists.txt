cmake_minimum_required(VERSION 3.8)

project(type-map)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include_directories(include)

option(TYPE_MAP_BUILD_TESTS "Build tests for type-map." ON)
if(TYPE_MAP_BUILD_TESTS)
	include_directories(external/Catch2/single_include)

	add_executable(test_tmap test/main.spec.cpp test/tmap.spec.cpp test/detail/hash_table.spec.cpp)
endif()

option(TYPE_MAP_BUILD_BENCHMARKS
	   "Build benchmarks for type-map." OFF)
if(TYPE_MAP_BUILD_BENCHMARKS)
	find_package(Threads REQUIRED)
	find_package(benchmark REQUIRED)

	include_directories(${benchmark_INCLUDE_DIRS})

	add_executable(bench_plist bench/main.cpp bench/tmap.cpp)
	target_link_libraries(bench_plist benchmark benchmark_main
						  Threads::Threads)
endif()
