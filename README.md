DESCRIPTION
===============
masked_xcorr efficiently computes the cross-correlation between two images, each of 
which can be independently masked, using fast Fourier techniques. In simple signal
processing terms, we are computing the cross-correlation of a 2D discrete signal with
a larger 2D discrete signal, while also allowing the user to specify arbitrarily shaped
supports --- henceforth referred to as masks --- for each signal. This computation
can be executed directly in the spatial domain, but the Fourier Transform makes it
possible to do this computation much faster. Our procedure follows the algorithms 
described in the following paper:

    Dirk Padfield. "Masked Object Registration in the Fourier Domain". 
    IEEE Transactions on Image Processing, vol.21(5), pp. 2706-2718, 2012. 

Dr. Padfield provides implementations in ITK and MATLAB. Our contribution is to 
provide an implementation that is compatible with OpenCV, and we hope to integrate
this code within OpenCV, as it addresses a need that is currently unmet.


INSTALLATION
===============
Requirements: opencv, boost, cmake 

On Mac OS X:

    # Homebrew instllation: http://brew.sh/
    brew install opencv boost cmake

On Debian systems:

    # OpenCV installation: https://help.ubuntu.com/community/OpenCV
    sudo apt-get install opencv boost-c++ cmake

On Windows systems:

    # It's the year 2013. Please learn how to use a real computer!

Procedure for building code using CMake:

    [masked_xcorr]$ mkdir build
    [masked_xcorr]$ cd build
    [masked_xcorr/build]$ cmake ..
    [masked_xcorr/build]$ make

Note that the last two commands are executed inside the build directory.

Afterwards, the tree structure should look like this:

    [~/masked_xcorr]$ tree -L 2
    .
    ├── CMakeLists.txt
    ├── README.md
    ├── build
    │   ├── CMakeCache.txt
    │   ├── CMakeFiles
    │   ├── Makefile
    │   ├── cmake_install.cmake
    │   └── src
    └── src
        ├── CMakeLists.txt
        ├── Doxyfile
        ├── masked_xcorr.cpp
        ├── normxcorr2_masked.cpp
        └── normxcorr2_masked.hpp

(For more information on using CMake, check out http://www.cmake.org/cmake/help/cmake_tutorial.html)

Now execute the program as follows:

    [masked_xcorr/build/src]$ ./masked_xcorr -c [fixedImage] -d [fixedMask] -e [movingImage] -f [movingMask]

Type ./masked_xcorr -h for a full help menu.



EXAMPLES
===============
In this example, we will use maximization of masked cross-correlations to locate the Batman logo in a scene.
The images we will use for this example are shown below, and can be downloaded as an archive <a href="http://qe-design.com/masked_xcorr/images/test_images.zip">here</a>.

fixedImage.jpg (scene):

![Screenshot](http://qe-design.com/masked_xcorr/images/fixedImage.jpg)

fixedMask.png (scene mask):

![Screenshot](http://qe-design.com/masked_xcorr/images/fixedMask.png)

movingImage.jpg (template):

![Screenshot](http://qe-design.com/masked_xcorr/images/movingImage.jpg)

movingMask.png (template mask):

![Screenshot](http://qe-design.com/masked_xcorr/images/movingMask.png)

Note that the Batman logo in the template does not exactly match the logo on Batman's chest in the scene.
Also, the Batman logo in the template is red outside of the oval logo, which would throw off a standard cross-correlator that uses rectangular supports.
The oval template mask allows us to ignore this red area.

Masked normalized cross-correlation using Fourier methods:

    [~/masked_xcorr/build/src]$ ./masked_xcorr -c fixedImage.jpg -d fixedMask.png -e movingImage.jpg -f movingMask.png -o xcorr.jpg -k 3
    Fixed image (scene): fixedImage.jpg: [690 x 800]
    Fixed mask (scene mask): fixedMask.png: [690 x 800]
    Moving image (template): movingImage.jpg: [104 x 63]
    Moving mask (template mask): movingMask.png: [104 x 63]
    Dimensions of combined image: 862 x 793
    Optimal larger dimensions for fast DFT: 864 x 800
        Calculated cross-correlation for one channel in 806.512 ms
        Calculated cross-correlation for one channel in 776.407 ms
        Calculated cross-correlation for one channel in 768.965 ms
    Total cross-correlation time for all channels: 2451.68 ms

    Results for channel 0 ...
    #1: 0.750705 (220,404)
    #2: 0.746034 (219,404)
    #3: 0.727210 (219,405)
    #4: 0.720789 (219,403)
    #5: 0.719367 (220,405)
    maximum value: 0.750705 @ (y,x)=(404,220)
    minimum value: -0.383996 @ (y,x)=(398,349)
    Writing image xcorr_0.jpg ... 
    Writing numerical matrices xcorr_0_correlation_matrix.tsv and xcorr_0_overlap_matrix.tsv ... 

    Results for channel 1 ...
    #1: 0.858291 (219,404)
    #2: 0.840575 (219,405)
    #3: 0.825339 (220,404)
    #4: 0.821008 (219,403)
    #5: 0.803989 (220,405)
    maximum value: 0.858291 @ (y,x)=(404,219)
    minimum value: -0.482819 @ (y,x)=(389,197)
    Writing image xcorr_1.jpg ... 
    Writing numerical matrices xcorr_1_correlation_matrix.tsv and xcorr_1_overlap_matrix.tsv ... 

    Results for channel 2 ...
    #1: 0.877009 (219,404)
    #2: 0.853381 (219,405)
    #3: 0.848980 (220,404)
    #4: 0.841817 (219,403)
    #5: 0.824849 (220,405)
    maximum value: 0.877009 @ (y,x)=(404,219)
    minimum value: -0.552982 @ (y,x)=(390,200)
    Writing image xcorr_2.jpg ... 
    Writing numerical matrices xcorr_2_correlation_matrix.tsv and xcorr_2_overlap_matrix.tsv ... 

    Writing blended correlation matrix xcorr.jpg ...
    #1: 0.827111 (219,404)
    #2: 0.808342 (220,404)
    #3: 0.807056 (219,405)
    #4: 0.794538 (219,403)
    #5: 0.782735 (220,405)
    maximum value: 0.827111 @ (y,x)=(404,219)
    minimum value: -0.441289 @ (y,x)=(398,347)

The resulting output images are shown below. Note that we ignore all areas where the intersection of the template mask with the scene mask has an area smaller than that of the template mask. 

xcorr_0.jpg (cross-correlation for blue channel):

![Screenshot](http://qe-design.com/masked_xcorr/images/xcorr_0.jpg)

xcorr_1.jpg (cross-correlation for green channel):

![Screenshot](http://qe-design.com/masked_xcorr/images/xcorr_1.jpg)

xcorr_2.jpg (cross-correlation for red channel):

![Screenshot](http://qe-design.com/masked_xcorr/images/xcorr_2.jpg)

Lastly, here is a composite image that averages the cross-correlation images from all three color channels:

xcorr.jpg (average of cross-correlation matrix from all 3 channels) 

![Screenshot](http://qe-design.com/masked_xcorr/images/xcorr.jpg)

- William Wu, 2013-09-30

