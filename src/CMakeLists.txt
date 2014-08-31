add_executable(
    masked_normxcorr
    masked_normxcorr.cpp
    normxcorr2_masked.cpp
    normxcorr2_masked.hpp
)
target_link_libraries(masked_normxcorr ${OpenCV_LIBS} ${Boost_LIBRARIES})
