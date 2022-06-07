SET (CMAKE_C_COMPILER             "clang")
SET (CMAKE_CXX_COMPILER            "clang++")
set(CMAKE_CXX_FLAGS "-Wall -std=c++11  ${EXTRA_FLAGS}")

cmake_minimum_required(VERSION 3.0)

project(gpstest)
   

set(SOURCES
	src/main.cpp
	src/GPSmgr.cpp
	src/MicroNMEA.cpp
	src/tranmerc.cpp
	src/utm.cpp
	src/I2C.cpp
	src/ErrorMgr/cpp
	)

 
add_executable(gpstest ${SOURCES})


set_target_properties(gpstest PROPERTIES
				CXX_STANDARD 17
				CXX_EXTENSIONS OFF
				)
 
set_target_properties(gpstest PROPERTIES
			OUTPUT_NAME "gpstest"
			)


target_link_libraries(gpstest  PRIVATE  gpiod ${EXTRA_LIBS} )

target_include_directories(gpstest
	PRIVATE
	src
)

set(CMAKE_BINARY_DIR "bin")
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
 
