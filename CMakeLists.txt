cmake_minimum_required(VERSION 3.10)
project(imsee CXX C)

set(DEMO_NAME "imsee_data_cap")
set(CMAKE_CXX_STANDARD 11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

macro(safe_set_static_flag)
    foreach(flag_var
        CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE
        CMAKE_CXX_FLAGS_MINSIZEREL CMAKE_CXX_FLAGS_RELWITHDEBINFO)
      if(${flag_var} MATCHES "/MD")
        string(REGEX REPLACE "/MD" "/MT" ${flag_var} "${${flag_var}}")
      endif(${flag_var} MATCHES "/MD")
    endforeach(flag_var)
endmacro()

set(USE_OPENCV 0)

if(USE_OPENCV) 
    add_definitions(-DUSE_OPENCV)
    set(OPENCV_DIR "/usr/local/lib/cmake/opencv4" CACHE PATH "Location of libraries")

    find_package(OpenCV REQUIRED PATHS ${OPENCV_DIR} NO_DEFAULT_PATH)

    message(STATUS "OpenCV lib: ${OpenCV_LIBS}")
else()
    message(STATUS "Not Use OpenCV")
endif()


include_directories("./driver/include")

add_executable(${DEMO_NAME} ./imsee_data_cap.cpp )

if(USE_OPENCV) 
    if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
	  message(STATUS "linux os")
	  if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
	    message(STATUS "aarch64")
	    target_link_libraries(${DEMO_NAME} ${OpenCV_LIBS} ${CMAKE_CURRENT_SOURCE_DIR}/driver/lib/aarch64/libusbdriver.so)
	  elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
	    message(STATUS "x86_64")
	    target_link_libraries(${DEMO_NAME} ${OpenCV_LIBS} ${CMAKE_CURRENT_SOURCE_DIR}/driver/lib/x86-64/libusbdriver.so)
	  endif()
	else()
	  message(STATUS "not support")
    endif()
else()
    if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
	  message(STATUS "linux os")
	  if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
	    message(STATUS "aarch64")
	    target_link_libraries(${DEMO_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/driver/lib/aarch64/libusbdriver.so)
	  elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
	    message(STATUS "x86_64")
	    target_link_libraries(${DEMO_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/driver/lib/x86-64/libusbdriver.so)
	  endif()
	else()
	  message(STATUS "not support")
    endif()
endif()



