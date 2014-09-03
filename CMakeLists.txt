cmake_minimum_required(VERSION 2.8)
project( masked_normxcorr )
set (masked_normxcorr_VERSION_MAJOR 1)
set (masked_normxcorr_VERSION_MINOR 0)
find_package( OpenCV REQUIRED )
find_package( Boost 1.42 REQUIRED COMPONENTS system filesystem program_options )
add_subdirectory( src )

set(env_script "${PROJECT_BINARY_DIR}/generated/TestEnv.cmake")
configure_file("cmake/TestEnv.cmake.in" "${env_script}" @ONLY)
set_directory_properties(PROPERTIES TEST_INCLUDE_FILE "${env_script}")

enable_testing()
add_test(
    NAME Simple
    COMMAND
        masked_normxcorr
        -c "${CMAKE_CURRENT_LIST_DIR}/data/fixedImage.jpg"
        -d "${CMAKE_CURRENT_LIST_DIR}/data/fixedMask.png"
        -e "${CMAKE_CURRENT_LIST_DIR}/data/movingImage.jpg"
        -f "${CMAKE_CURRENT_LIST_DIR}/data/movingMask.png"
)
