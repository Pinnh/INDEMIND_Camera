# Copyright 2020 Indemind Co., Ltd. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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



