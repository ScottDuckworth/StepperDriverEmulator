include(FetchContent)

FetchContent_Declare(
    unity
    GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git
    GIT_TAG        v2.6.0
)
FetchContent_MakeAvailable(unity)

add_compile_definitions(UNITY_SUPPORT_64)
add_compile_definitions(UNIT_TEST)

# 1. Position Tracking & Coordinates Test
add_executable(test_position
    test_position.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Core/Src/position.c
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

# 2. Motion Physics & Step Planning Test
add_executable(test_motion
    test_motion.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Core/Src/motion.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Core/Src/quadrature.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Core/Src/config_store.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Core/Src/mathutil.c
)
target_include_directories(test_motion PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../Core/Inc
)
target_link_libraries(test_motion PRIVATE
    unity
)
if(UNIX)
    target_link_libraries(test_motion PRIVATE m)
endif()
add_test(NAME test_motion COMMAND $<TARGET_FILE:test_motion> WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

# 3. Serial CLI Command Parser Test
add_executable(test_cmd
    test_cmd.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Core/Src/cmd.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Core/Src/mathutil.c
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

# 6. Configuration Storage & Persistence Test
add_executable(test_config_store
    test_config_store.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Core/Src/config_store.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Core/Src/mathutil.c
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

# 7. Math Utilities (GCD & Float Ratio) Test
add_executable(test_mathutil
    test_mathutil.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Core/Src/mathutil.c
)
target_include_directories(test_mathutil PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../Core/Inc
)
target_link_libraries(test_mathutil PRIVATE
    unity
)
if(UNIX)
    target_link_libraries(test_mathutil PRIVATE m)
endif()
add_test(NAME test_mathutil COMMAND $<TARGET_FILE:test_mathutil> WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
