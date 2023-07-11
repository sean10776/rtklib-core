cmake_minimum_required(VERSION 3.15)

project(tests LANGUAGES C)

# find all test source files
file(GLOB TESTS_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.c)

# copy data folder to build folder
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

# create a test for each source file
foreach(test_source ${TESTS_FILES})
    get_filename_component(test_name ${test_source} NAME_WE)

    add_executable(${test_name} ${test_source})

    target_link_libraries(${test_name} rtklib-core)

    message(STATUS "Adding test ${test_name}")
endforeach()
