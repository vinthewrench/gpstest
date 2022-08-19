SET (CMAKE_C_COMPILER             "clang")
SET (CMAKE_CXX_COMPILER            "clang++")
set(CMAKE_CXX_FLAGS "-Wall -std=c++11  ${EXTRA_FLAGS}")

cmake_minimum_required(VERSION 3.0)

project(gpstest)
   
find_package(Threads)

set(SOURCES
	src/main.cpp
	src/GPSmgr.cpp
	src/minmea.cpp
	src/tranmerc.cpp
	src/utm.cpp
 	src/ErrorMgr.cpp
	)

 
add_executable(gpstest ${SOURCES})


set_target_properties(gpstest PROPERTIES
				CXX_STANDARD 17
				CXX_EXTENSIONS OFF
				)
 
set_target_properties(gpstest PROPERTIES
			OUTPUT_NAME "gpstest"
			)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

target_link_libraries(gpstest  PRIVATE   Threads::Threads  ${EXTRA_LIBS} )

target_include_directories(gpstest
	PRIVATE
	src
)

set(CMAKE_BINARY_DIR "bin")
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
 
