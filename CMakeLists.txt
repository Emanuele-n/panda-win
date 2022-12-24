cmake_minimum_required(VERSION 3.5)

project(panda_win VERSION 0.1 LANGUAGES C CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# FYI
# CMAKE_INCLUDE_PATH: Semicolon-separated list of directories specifying a search path for the find_file() and find_path() commands
# CMAKE_LIBRARY_PATH: Semicolon-separated list of directories specifying a search path for the find_library() command.
# CMAKE_PREFIX_PATH:  " " directories specifying installation prefixes to be searched by the find_package(), find_program(), find_library(), find_file(), and find_path() commands.

# Locate and find boost lib
set(Boost_USE_STATIC_LIBS ON) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost REQUIRED)

# Locate and find franka lib
SET(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${CMAKE_CURRENT_LIST_DIR}/libs/libfranka/build)
find_package(franka REQUIRED)

# Source files
set(SRC_EXPR
    ${CMAKE_CURRENT_LIST_DIR}/src/main.c*
    ${CMAKE_CURRENT_LIST_DIR}/src/aProxy/*.c*
    ${CMAKE_CURRENT_LIST_DIR}/src/bController/*.c*
    ${CMAKE_CURRENT_LIST_DIR}/src/cEntity/*.c*
    ${CMAKE_CURRENT_LIST_DIR}/src/dDBWrapper/*.c*
    ${CMAKE_CURRENT_LIST_DIR}/src/eUtils/*.c*
    ${CMAKE_CURRENT_LIST_DIR}/libs/FrankaPandaDynModelLib/*.c*
    "C:/Program Files/CoppeliaRobotics/CoppeliaSimEdu/programming/legacyRemoteApi/remoteApi/extApi.c"
    "C:/Program Files/CoppeliaRobotics/CoppeliaSimEdu/programming/legacyRemoteApi/remoteApi/extApiPlatform.c"
    "C:/Program Files/CoppeliaRobotics/CoppeliaSimEdu/programming/common/shared_memory.c"
)

# Header files
set(HEADER_EXPR
    ${CMAKE_CURRENT_LIST_DIR}/include/aProxy/*.h*
    ${CMAKE_CURRENT_LIST_DIR}/include/bController/*.h*
    ${CMAKE_CURRENT_LIST_DIR}/include/cEntity/*.h*
    ${CMAKE_CURRENT_LIST_DIR}/include/dDBWrapper/*.h*
    ${CMAKE_CURRENT_LIST_DIR}/include/eUtils/*.h*
    ${CMAKE_CURRENT_LIST_DIR}/libs/FrankaPandaDynModelLib/*.h*
    "C:/Program Files/CoppeliaRobotics/CoppeliaSimEdu/programming/legacyRemoteApi/remoteApi/extApi.h"
    "C:/Program Files/CoppeliaRobotics/CoppeliaSimEdu/programming/legacyRemoteApi/remoteApi/extApiPlatform.h"
    "C:/Program Files/CoppeliaRobotics/CoppeliaSimEdu/programming/common/shared_memory.h"
)

file(GLOB HEADERS ${HEADER_EXPR})
file(GLOB SRCS ${SRC_EXPR})

# Add the given directories to those the compiler uses to search for include files
include_directories(
    ${CMAKE_CURRENT_LIST_DIR}/include/aProxy
    ${CMAKE_CURRENT_LIST_DIR}/include/bController
    ${CMAKE_CURRENT_LIST_DIR}/include/cEntity
    ${CMAKE_CURRENT_LIST_DIR}/include/dDBWrapper
    ${CMAKE_CURRENT_LIST_DIR}/include/eUtils
    ${CMAKE_CURRENT_LIST_DIR}/libs/FrankaPandaDynModelLib
    "C:/Program Files/CoppeliaRobotics/CoppeliaSimEdu/programming/include"
    "C:/Program Files/CoppeliaRobotics/CoppeliaSimEdu/programming/legacyRemoteApi/remoteApi"
    ${Boost_INCLUDE_DIRS}
    ${Franka_INCLUDE_DIRS}
)

if(MSVC)
    add_definitions(-D_CRT_SECURE_NO_WARNINGS -DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255 -D_SILENCE_EXPERIMENTAL_FILESYSTEM_DEPRECATION_WARNING)
endif()

add_compile_definitions(PROJECT_NAME)
add_compile_definitions(_SILENCE_ALL_CXX17_DEPRECATION_WARNINGS)
add_compile_definitions(BOOST_CONFIG_SUPPRESS_OUTDATED_MESSAGE) 
add_compile_definitions(BOOST_BIND_GLOBAL_PLACEHOLDERS)

# Build
add_executable(${PROJECT_NAME} ${HEADERS} ${SRCS})

# Link
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /SUBSYSTEM:CONSOLE /ENTRY:mainCRTStartup")
message(${Franka_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${Franka_LIBRARIES})
link_directories( ${CMAKE_CURRENT_LIST_DIR}/libs/libfranka/build/Release)
target_link_options(${PROJECT_NAME} PUBLIC $<$<CXX_COMPILER_ID:MSVC>:/FORCE:MULTIPLE>)

set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
    MACOSX_BUNDLE_GUI_IDENTIFIER my.example.com
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)
