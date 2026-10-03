# Prefer an installed GoogleTest (vcpkg, apt, etc.); otherwise download it.
find_package(GTest 1.12 QUIET)

if(NOT GTest_FOUND)
    message(STATUS "GoogleTest not found locally - fetching it")
    include(FetchContent)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG        v1.15.2
        GIT_SHALLOW    TRUE)
    # Use the same runtime library as the rest of the project on Windows.
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
endif()

add_executable(tracker_tests
    test_matrix.cpp
    test_kalman_filter.cpp)
target_link_libraries(tracker_tests PRIVATE tracker tracker_warnings GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(tracker_tests)
