cmake_minimum_required(VERSION 3.15)

project(rtklib_core LANGUAGES C)

set(RTKLIB_VERSION b34g)
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
message(${PROJECT_NAME} " " ${RTKLIB_VERSION})
option(BUILD_TESTS "Build tests" ON)

aux_source_directory(${SRC_DIR} RTKLIB_SRC)
aux_source_directory(${SRC_DIR}/rcv RTKLIB_SRC_RCV)

# Remove comnav.c and tersus.c from RTKLIB_SRC_RCV, cause they are deprecated
set(FILES_TO_REMOVE
    ${SRC_DIR}/rcv/comnav.c
    ${SRC_DIR}/rcv/tersus.c
)
list(REMOVE_ITEM RTKLIB_SRC_RCV ${FILES_TO_REMOVE})

if(WIN32)
    add_compile_definitions(-DWIN32)
endif()

if(BUILD_TESTS)
    add_compile_options(-g)
    add_compile_definitions(-DNFREQ=3 -DNEXOBS=3 -DENAGLO -DENAGAL -DENACMP -DTRACE)
endif()

add_library(${PROJECT_NAME} STATIC ${RTKLIB_SRC} ${RTKLIB_SRC_RCV})

set(RTKLIB_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
include_directories(${RTKLIB_INCLUDE_DIR})
if(WIN32)
    target_link_libraries(${PROJECT_NAME} wsock32 winmm)
endif()

if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
    message("Tests enabled")
endif()