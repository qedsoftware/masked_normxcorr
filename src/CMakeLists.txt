include_directories (${masked_xcorr_SOURCE_DIR}/src) 
link_directories (${masked_xcorr_BINARY_DIR}/src)
add_executable( masked_xcorr masked_xcorr.cpp normxcorr2_masked.cpp) 
target_link_libraries( masked_xcorr ${OpenCV_LIBS} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY}) 

