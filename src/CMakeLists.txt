include_directories (${masked_normxcorr_SOURCE_DIR}/src) 
link_directories (${masked_normxcorr_BINARY_DIR}/src)
add_executable( masked_normxcorr masked_normxcorr.cpp normxcorr2_masked.cpp) 
target_link_libraries( masked_normxcorr ${OpenCV_LIBS} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY}) 

