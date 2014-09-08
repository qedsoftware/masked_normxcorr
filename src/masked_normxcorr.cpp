/***********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright 2013 The Math Path Inc.
 * DBA: Quantitative Engineering Design (http://qe-design.com)
 * Authors: William Wu, Jiehua Chen, Zhang Zhiming, Michał Łazowik
 * Primary Contact: William Wu (william.wu@qe-design.com)
 *
 * THE BSD LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *************************************************************************/

/** @mainpage Masked Cross-Correlation in the Fourier Domain

@section purpose Purpose/Overview

This program efficiently computes the cross-correlation between two images,
each of which can be independently masked, using fast Fourier techniques. Our
procedure follows the algorithms described in the following paper:

Dirk Padfield. "Masked Object Registration in the Fourier Domain". IEEE Transactions on Image Processing, vol.21(5), pp. 2706-2718, 2012.

@section reqs Requirements

The default input image files are fixedImage.jpg and movingImage.jpg.

The default input masked image files are fixedMask.png and movingMask.png.

The output image is xcorr_image.jpg which shows the relative intensity of one channel's masked correlation.

The application shall display the xcorr_image.jpg on screen.

@section globals Global Data/Functions

There is no global data in this application.  All the data is contained within objects.

@section objects Objects

See the individual object documentation pages for more info on each object.

Xcorr_opencv - A class calculating the masked correlation in the fourier domain

@section arch High-level architecture

The general workflow of the code is:

1. Read input files.
2. Pass files to class Xcorr_opencv.
3. For each channel, calculate masked correlation in Fourier domain.
4. Get the relative intensity of masked correlation on one channel.
5. Display the relative matching intensity on the screen.

@section functions Function Descriptions

All functions are described in detail on the documentation pages for the classes
that either contain them or are friends of them.

@section ui User Interface

The only user interface is that the user will be prompted a image for showing the relative matching intensity on one channel. The user need to switch focus to the display window and press any key to continue.

@section testing Testing

 A shining point shall be shown on the top middle of the image, which means that movingImage.jpg matches the head of fixedImage.jpg.
 */
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <vector>
#include <numeric>
#include <fstream>
#include <functional>

#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>

namespace fs = boost::filesystem;

#include "normxcorr2_masked.hpp"

using namespace cv;

struct Mat_elt {
    int i;
    int j;
    double value;
};
bool Mat_elt_cmp(const Mat_elt& x, const Mat_elt& y) { return (x.value > y.value); }

int
is_file_openable (const std::string& filename) {
    ifstream my_file(filename.c_str(), ios::in);
    return my_file.good();
}

/*
Write an image of the cross-correlation matrix.
The Mat::convertTo() method is restricted to mappings of the type: x --> a*x + b, where we can specify a and b.
For maximum dynamic range:
    min --> 0
    max --> 255
Linear equations:
    (1): a(min) + b = 0
    (2): a(max) + b = 255
Solving:
    (1): b  = -a*min
    (2): a(max) - a(min) = 255
Therefore,
    a = 255 / (max - min)
    b = - 255 * min / (max - min)
Note 1: If we do not wish to achieve maximum dynamic range, and simply want to map -1 --> 0 and +1 --> 255, then a = b = 127.5.
Note 2: The resulting image does NOT match the output of the interactive window. I don't understand yet what kind of autoscaling imshow is doing.
*/
void
write_scaled_image(cv::Mat mat, const char* filename) {
    double minVal, maxVal;
    Point minPoint, maxPoint;
    minMaxLoc(mat, &minVal, &maxVal, &minPoint, &maxPoint, noArray());
    cv::Mat mat_scaled;
    double a = 255 / (maxVal - minVal);
    double b = -255 * minVal / (maxVal - minVal);
    mat.convertTo(mat_scaled, CV_8UC1, a, b);
    imwrite(filename, mat_scaled);
}

void
statistics(cv::Mat CC, int topK) {
    std::vector<Mat_elt> data;
    for (int i=0; i<CC.rows; i++) {
        for (int j=0; j<CC.cols; j++) {
            Mat_elt e;
            e.i = i;
            e.j = j;
            e.value = CC.at<double>(i,j);
            data.push_back(e);
        }
    }
    /* find top K values, and minimum */
    std::sort(data.begin(), data.end(), Mat_elt_cmp);
    for (int k=0; k<topK; k++) {
        printf("#%d: %f (%d,%d)\n", k+1, data[k].value, data[k].i, data[k].j);
    }
    double maxVal = data.front().value;
    double minVal = data.back().value;
    printf("maximum value: %f @ (y,x)=(%d,%d)\n", maxVal, data.front().j, data.front().i);
    printf("minimum value: %f @ (y,x)=(%d,%d)\n", minVal, data.back().j, data.back().i);
}

template <typename T>
void
validateRange(T value, T min, T max, const std::string &option_name = "") {
    if (value < min || value > max) {
        throw boost::program_options::validation_error(
            boost::program_options::validation_error::invalid_option_value,
            option_name
        );
    }
}

/**
  Main method

  Get image file names and pass them to the member function of Xcorr_opencv.
  After the relative intensities of masked correlation are calculated,
  main function shall get the result of one channel and display the result in
  one image.

  @return The exit status of the program as an integer
*/
int
main (int argc, char **argv)
{
    namespace po = boost::program_options;

    po::options_description desc(
        "Fast Masked Cross-Correlation Using Fourier Domain Methods\n\n"
        "Remarks:\n"
        "    Pixels in fixedImage that do not fall under the fixedMask "
        "are ignored in all correlation computations.\n"
        "    Pixels in movingImage that do not fall under the movingMask "
        "are ignored in all correlation computations.\n"
        "    If both overlap-fraction and overlap-pixels are present a "
        "greater resulting number is used"
    );

    std::string fixedImageName;
    std::string fixedMaskName;
    std::string movingImageName;
    std::string movingMaskName;
    std::string outputImageName;
    double requiredFractionOfOverlappingPixels;
    double requiredNumberOfOverlappingPixels;
    int topK;

    desc.add_options()
        ("help,h", "show help message")
        (
            "fixed-image,F",
            po::value<std::string>(&fixedImageName)->
                required(),
            "scene that we wish to search for the template"
        )
        (
            "fixed-mask,f",
            po::value<std::string>(&fixedMaskName)->
                required(),
            "binary mask specifying the search region in the scene, "
            "having the same dimensions as fixedImage"
        )
        (
            "moving-image,M",
            po::value<std::string>(&movingImageName)->
                required(),
            "template that we 'slide' throughout the scene"
        )
        (
            "moving-mask,m",
            po::value<std::string>(&movingMaskName)->
                required(),
            "a binary mask specifying the region of the template not "
            "to be ignored, having the same dimensions as movingImage"
        )
        (
            "overlap-fraction,o",
            po::value<double>(&requiredFractionOfOverlappingPixels)->
                default_value(0.3, "0.3")->notifier(
                    boost::bind(&validateRange<double>, _1, 0.0, 1.0, "\u2012O [ overlap\u2012fraction ]")
                ),
            "required fraction of maximum possible number of common pixels of masked images"
        )
        (
            "overlap-pixels,p",
            po::value<double>(&requiredNumberOfOverlappingPixels)->
                default_value(0),
            "required number of common pixels of masked images"
        )
        (
            "output,O",
            po::value<std::string>(&outputImageName)->
                default_value("xcorr.jpg"),
            "output image"
        )
        (
            "top-k,k",
            po::value<int>(&topK)->default_value(5),
            "print locations of the top k highest normalized "
            "cross-correlations and their values"
        )
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return EXIT_SUCCESS;
    }

    try {
        po::notify(vm);
    } catch (boost::program_options::error &e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    if (!is_file_openable(fixedImageName)) {
        std::cerr << "ERROR: Cannot open file " << fixedImageName << std::endl;
        return EXIT_FAILURE;
    }

    if (!is_file_openable(fixedMaskName)) {
        std::cerr << "ERROR: Cannot open file " << fixedMaskName << std::endl;
        return EXIT_FAILURE;
    }

    if (!is_file_openable(movingMaskName)) {
        std::cerr << "ERROR: Cannot open file " << movingMaskName << std::endl;
        return EXIT_FAILURE;
    }

    if (!is_file_openable(movingImageName)) {
        std::cerr << "ERROR: Cannot open file " << movingImageName << std::endl;
        return EXIT_FAILURE;
    }

    fs::path pathname(outputImageName);
    std::string dirname  = pathname.parent_path().string();
    std::string basename = pathname.stem().string();
    std::string extension = pathname.extension().string();
    if (extension.length() == 0) {
        extension = ".jpg"; // default
    }

    Xcorr_opencv *p_xcorr = new Xcorr_opencv;
    p_xcorr->Initialization(
        string(fixedImageName),
        string(fixedMaskName),
        string(movingImageName),
        string(movingMaskName),
        requiredFractionOfOverlappingPixels,
        requiredNumberOfOverlappingPixels
    );

    double t = (double)getTickCount();
    p_xcorr->CalXcorr();
    t = (double)getTickCount() - t;
    printf("Total cross-correlation time for all channels: %g ms\n\n", t*1000/getTickFrequency());
    t = (double)getTickCount();

    vector<cv::Mat> CC_vec;
    vector<cv::Mat> overlap_vec;
    cv::Mat CC;
    cv::Mat overlap;
    cv::Mat CC_blend;

    // iterate through channels
    for (int ch=0; ch<3; ch++) {
        printf("Results for channel %d ...\n",ch);
        p_xcorr->GetResult(CC,overlap,ch);
        CC_vec.push_back(CC);
        overlap_vec.push_back(overlap);

        // // Display xcorr matrix using an interactive window
        // cv::namedWindow("xcorr matrix", CV_WINDOW_AUTOSIZE);
        // cv::imshow("xcorr matrix", CC);
        // std::cout << "Displaying xcorr matrix. Switch focus to the display window and press any key to continue ..." << std::endl;
        // cv::waitKey();

        // find top matches
        statistics(CC, topK);

        // write xcorr image to disk
        std::string outputChannelImageBasename;
        if (dirname.length() > 0) {
            outputChannelImageBasename = dirname + "/" + basename + "_" + boost::lexical_cast<string>(ch);
        } else {
            outputChannelImageBasename = basename + "_" + boost::lexical_cast<string>(ch);
        }
        cout << "Writing image " << outputChannelImageBasename + extension << " ... " << endl;
        write_scaled_image(CC, (outputChannelImageBasename + extension).c_str());

        // writing xcorr matrices to disk
        std::string xcorrMatrixChannelName = (outputChannelImageBasename + "_correlation_matrix.tsv");
        std::string overlapMatrixChannelName = (outputChannelImageBasename + "_overlap_matrix.tsv");
        FILE* ccfile = fopen(xcorrMatrixChannelName.c_str(),"w+t"); // overwrite mode
        FILE* overlapfile = fopen(overlapMatrixChannelName.c_str(),"w+t");
        cout << "Writing numerical matrices " << xcorrMatrixChannelName << " and " << overlapMatrixChannelName << " ... " << endl;
        for (int i = 0; i < CC.rows; i++)
        {
            for (int j = 0; j < CC.cols; j++)
            {
                fprintf(ccfile,"%0.6f\t",CC.at<double>(i,j));
                fprintf(overlapfile,"%0.0f\t",overlap.at<double>(i,j));
            }
            fprintf(ccfile,"\n");
            fprintf(overlapfile,"\n");
        }
        fclose(ccfile);
        fclose(overlapfile);

        printf("\n");
    }
    std::string outputBlendedImageName;
    if (dirname.length() > 0) {
        outputBlendedImageName = dirname + "/" + basename + extension;
    } else {
        outputBlendedImageName = basename + extension;
    }
    cout << "Writing blended correlation matrix " << outputBlendedImageName << " ..." << endl;
    addWeighted(CC_vec[0],1.0/3,CC_vec[1],1.0/3,0,CC_blend);
    addWeighted(CC_blend,1.0,CC_vec[2],1.0/3,0,CC_blend);
    write_scaled_image(CC_blend, outputBlendedImageName.c_str());
    statistics(CC_blend, topK);

    return 0;
}
