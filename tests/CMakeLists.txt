include(FetchContent)

FetchContent_Declare(
    unity
    GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git
    GIT_TAG        v2.6.0
)
FetchContent_MakeAvailable(unity)

add_compile_definitions(UNITY_SUPPORT_64)

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
if(UNIX)
    target_link_libraries(test_coordinates PRIVATE m)
endif()
add_test(NAME test_coordinates COMMAND $<TARGET_FILE:test_coordinates> WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

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
if(UNIX)
    target_link_libraries(test_physics PRIVATE m)
endif()
add_test(NAME test_physics COMMAND $<TARGET_FILE:test_physics> WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

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
if(UNIX)
    target_link_libraries(test_cmd PRIVATE m)
endif()
add_test(NAME test_cmd COMMAND $<TARGET_FILE:test_cmd> WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

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
if(UNIX)
    target_link_libraries(test_quadrature PRIVATE m)
endif()
add_test(NAME test_quadrature COMMAND $<TARGET_FILE:test_quadrature> WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

# 5. Status LED & Priority Engine Test
add_executable(test_led
    test_led.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Core/Src/led_control.c
)
target_include_directories(test_led PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../Core/Inc
)
target_link_libraries(test_led PRIVATE
    unity
)
if(UNIX)
    target_link_libraries(test_led PRIVATE m)
endif()
add_test(NAME test_led COMMAND $<TARGET_FILE:test_led> WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

# 6. Position Tracking Math Test
add_executable(test_position
    test_position.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Core/Src/position_tracker.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Core/Src/motion_math.c
)
target_include_directories(test_position PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../Core/Inc
)
target_link_libraries(test_position PRIVATE
    unity
)
if(UNIX)
    target_link_libraries(test_position PRIVATE m)
endif()
add_test(NAME test_position COMMAND $<TARGET_FILE:test_position> WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

# 7. Motion Planning & Stall Engine Test
add_executable(test_motion_planner
    test_motion_planner.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Core/Src/motion_planner.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Core/Src/motion_math.c
)
target_include_directories(test_motion_planner PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../Core/Inc
)
target_link_libraries(test_motion_planner PRIVATE
    unity
)
if(UNIX)
    target_link_libraries(test_motion_planner PRIVATE m)
endif()
add_test(NAME test_motion_planner COMMAND $<TARGET_FILE:test_motion_planner> WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

# 8. Configuration Storage & Persistence Test
add_executable(test_config_store
    test_config_store.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Core/Src/config_store.c
)
target_compile_definitions(test_config_store PRIVATE
    UNIT_TEST
)
target_include_directories(test_config_store PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../Core/Inc
)
target_link_libraries(test_config_store PRIVATE
    unity
)
if(UNIX)
    target_link_libraries(test_config_store PRIVATE m)
endif()
add_test(NAME test_config_store COMMAND $<TARGET_FILE:test_config_store> WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})


