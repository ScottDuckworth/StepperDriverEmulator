include(FetchContent)

FetchContent_Declare(
    unity
    GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git
    GIT_TAG        v2.6.0
)
FetchContent_MakeAvailable(unity)

add_executable(test_coordinates
    test_coordinates.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Core/Src/motion_math.c
)

target_include_directories(test_coordinates PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../Core/Inc
)

target_link_libraries(test_coordinates PRIVATE
    unity
)

add_test(NAME test_coordinates COMMAND test_coordinates)
