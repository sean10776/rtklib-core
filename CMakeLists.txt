cmake_minimum_required(VERSION 3.15)

project(rtklib_core LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(RTKLIB_VERSION b34g)
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

message(${PROJECT_NAME} " " ${RTKLIB_VERSION})

option(BUILD_TESTS "Build tests" ON)

aux_source_directory(${SRC_DIR} RTKLIB_SRC)
aux_source_directory(${SRC_DIR}/rcv RTKLIB_SRC_RCV)

# Remove deprecated files
list(REMOVE_ITEM RTKLIB_SRC_RCV
    ${SRC_DIR}/rcv/comnav.c
    ${SRC_DIR}/rcv/tersus.c
)

if(WIN32)
    add_compile_definitions(WIN32)
    if(MSVC)
        add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
    endif()
elseif(CYGWIN)
    add_compile_options(-g)
endif()

if(BUILD_TESTS)
    add_compile_definitions(NFREQ=3 NEXOBS=3 ENAGLO ENAGAL ENACMP TRACE)
endif()

add_library(${PROJECT_NAME} STATIC ${RTKLIB_SRC} ${RTKLIB_SRC_RCV})

set(RTKLIB_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_include_directories(${PROJECT_NAME} PUBLIC ${RTKLIB_INCLUDE_DIR})

if(WIN32)
    target_link_libraries(${PROJECT_NAME} wsock32 winmm)
elseif(UNIX)
    target_link_libraries(${PROJECT_NAME} m rt pthread)
endif()

if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
    message("Tests enabled")
endif()