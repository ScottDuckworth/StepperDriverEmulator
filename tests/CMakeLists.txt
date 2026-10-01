include(FetchContent)

FetchContent_Declare(
    unity
    GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git
    GIT_TAG        v2.6.0
)
FetchContent_MakeAvailable(unity)

# 1. Coordinate Math Test
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

# 2. Motor Physics & Dynamics Test
add_executable(test_physics
    test_physics.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Core/Src/motion_math.c
)
target_include_directories(test_physics PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../Core/Inc
)
target_link_libraries(test_physics PRIVATE
    unity
)
add_test(NAME test_physics COMMAND test_physics)

# 3. Serial CLI Command Parser Test
add_executable(test_cmd
    test_cmd.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Core/Src/cmd.c
)
target_compile_definitions(test_cmd PRIVATE
    UNIT_TEST
)
target_include_directories(test_cmd PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../Core/Inc
)
target_link_libraries(test_cmd PRIVATE
    unity
)
add_test(NAME test_cmd COMMAND test_cmd)

# 4. Quadrature Generation & Pacing Test
add_executable(test_quadrature
    test_quadrature.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Core/Src/quadrature.c
)
target_include_directories(test_quadrature PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../Core/Inc
)
target_link_libraries(test_quadrature PRIVATE
    unity
)
add_test(NAME test_quadrature COMMAND test_quadrature)

