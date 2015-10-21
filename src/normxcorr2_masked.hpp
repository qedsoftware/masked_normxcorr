/***********************************************************************
 * Software License Agreement (BSD License)
 *
 * (c) 2013 Quantitative Engineering Design (http://qed.ai)
 * Authors: William Wu, Jiehua Chen, Zhang Zhiming, Michał Łazowik
 * Primary Contact: William Wu (w@qed.ai)
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

#ifndef  _OPENCV_NORMXCORR2_MASKED_HPP
#define  _OPENCV_NORMXCORR2_MASKED_HPP
#ifdef __cplusplus

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv.h>
#include <iostream>
#include <vector>

namespace cv {
#define FFT_SIGN_TtoF 1
#define FFT_SIGN_FtoT -1
#define EPS 0.0000000000000002204

using namespace std;
using namespace cv;

/**
  A class for efficiently calculating the cross-correlation of masked images by
  using Fourier domain techniques. Based on theory described by D. Padfield, "Masked 
  Object Registration in the Fourier Domain". IEEE Transactions on Image Processing, 
  vol.21(5), pp. 2706-2718, 2012.

  @authors William Wu, Zhiming Zhang, Jiehua Chen
*/

class Xcorr_opencv {
    public:
        /** 
          Xcorr_opencv constructor
          @post eps is initialized.
          */
        Xcorr_opencv();

        /** Xcorr_opencv destructor */
        ~Xcorr_opencv();

        /** The image for registering */
        cv::Mat fixedImage;
        /** The image mask for fixedImage */
        cv::Mat fixedMask;
        /** The image for registered */
        cv::Mat movingImage;
        /** The image mask for movingImage */
        cv::Mat movingMask;

        double requiredFractionOfOverlappingPixels;
        double requiredNumberOfOverlappingPixels;

        /**
          Channel number of the images for calculating relative intensity
          of masked correlation
         */
        int channelnum;

        /** Channels of splitted fixedImage. (sbgr_fixedImage[0] = blue channel of fixedImage.) */
        vector<cv::Mat> sbgr_fixedImage;
        /** Channels of splitted fixedMask. */
        vector<cv::Mat> sbgr_fixedMask;
        /** Channels of splitted movingImage. */
        vector<cv::Mat> sbgr_movingImage;
        /** Channels of splitted movingMask. */
        vector<cv::Mat> sbgr_movingMask;

        /** 
          Scaled fixedImage with optimized size. 
          The gap is padded with zeros.
         */
        cv::Mat optFixedImage;
        /** Scaled fixedMasked with optimized size. */
        cv::Mat optFixedMask;
        /** Scaled movingImage with optimized size. */
        cv::Mat optMovingImage;
        /** Scaled movingMasked with optimized size. */
        cv::Mat optMovingMask;
        /** Squared optFixedImage. */
        cv::Mat optFixedSquared;
        /** Squared optMovingImage. */
        cv::Mat optMovingSquared;

        /** FFT of optFixedImage*/
        IplImage *optFixedImage_FFT;
        /** FFT of optFixedMask*/
        IplImage *optFixedMask_FFT;
        /** FFT of optMovingImage*/
        IplImage *optMovingImage_FFT;
        /** FFT of optMovingMask*/
        IplImage *optMovingMask_FFT;
        /** FFT of optFixedSquared*/
        IplImage *optFixedSquared_FFT;
        /** FFT of optMovingSquared*/
        IplImage *optMovingSquared_FFT;

        /** 
          Relative intensity of all channels' masked correlation. 
          One element of C_result stores the NCC matrix for one channel.
         */
        vector<cv::Mat> C_result;
        /** Number of overlap masked pixels. */
        vector<cv::Mat> numberOfOverlapMaskedPixels_result;

        /** The combined size of fixedImage and movingImage*/
        int combinedSize[2];
        /** The optimal size of fixedImage and movingImage*/
        int optimalSize[2];
        /** CvSize-type variable for the optimal size */
        CvSize optimalCvsize;
        double fnorm;
        /** The smallest non-zero positive number. */
        double eps;

        /** 
         Divides numerator matrix by denominator matrix, scales results to [-1,1], and accounts for small overlap phenomenon. 
         */
        int PostProcessing(cv::Mat &matC, cv::Mat &matNumerator, cv::Mat &matDenom, double tol, double minimum, double maximum, cv::Mat &numberOfOverlapMaskedPixels, double minimumOverlapSize);

        /**
          Divide matNumerator by matDenom when the element of matDenom is nonzero; 
          otherwise, the corresponding element of matNumerator is unchanged. 
          Return the relative intensity of masked correlation in matC.

          @param matc The relative intensity of masked correlation
          @param matNumerator The convolution of movingImage and fixedImage
          @param matDenom The product of fixedDenom and movingDenom. fixedDenom is the convolution of movingMask and squared fixedImage. movingDenom is the convolution of fixedMask and squared movingImage.
          @param tol (tolerance) is the minimum positive number. Any number whose absolute value is below tol shall be considered zero.
         */
        int DivideNonZeroElem(cv::Mat &matC, cv::Mat &matNumerator, cv::Mat &matDenom, double tol);

        /**
          Return sum of all entries in matrix.
          @param matImage The matrix of an image
         */
        double MatrixSum(cv::Mat &matImage);

        /**
          Return maximum absolute value of all entries in matrix.
          @param matImage The matrix of an image
         */
        double MaxAbsValue(cv::Mat &matImage);

        /**
          Scan the matrix and compare each element with minimum. If the element is less than minimum, the element shall be assigned to minimum. Return the result in matImage.
          @param matImage The matrix of a image
          @param minimum The lower limit of the matrix element
         */
        int ThresholdLower(cv::Mat &matImage, double minimum);

        /**
          Scan the matrix and compare each element with maximum. If the element is larger than maximum, the element shall be assigned to maximum. Return the result in matImage.
          @param matImage The matrix of an image
          @param maximum The upper limit of the matrix element
         */
        int ThresholdUpper(cv::Mat &matImage, double maximum);
        /**
          Initializing some member variables, such as:
          1.  fixedImage
          2.  fixedMask
          3.  movingImage
          4.  movingMask
          5.  requiredFractionOfOverlappingPixels
          6.  channelnum
          7.  combinedSize
          8.  optimalSize
          9.  sbgr_fixedImage
          10.  sbgr_fixedMask
          11. sbgr_movingImage
          12. sbgr_movingMask
          13. All matrices of images with optimized dimension

          @param fixedImageName The file name of fixed image
          @param fixedMaskName The file name of fixedMask image
          @param movingImageName The file name of moving image
          @param movingMaskName The file name of movingMask image
          @param requiredFractionOfOverlappingPixels required fraction of overlapping pixels
         */
        int Initialization(
            string fixedImageName,
            string fixedMaskName,
            string movingImageName,
            string movingMaskName,
            double requiredFractionOfOverlappingPixels,
            double requiredNumberOfOverlappingPixels
        );
        /**
          Calculate the masked correlations of all channels.
          @pre Initialization must be executed first.
          @post The results are stored in C_result and numberOfOverlapMaskedPixels_result.
         */
        int CalXcorr();

        /**
          Calculate the masked correlation of one channel.
          @post The results are stored in C_result and numberOfOverlapMaskedPixels_result.
          @param curChannel The current channel number to be calculated.
         */
        int CalculateOneChannelXcorr(int curChannel);
        /**
          Call round function on each element of the matrix. Return the result in matImage.
          @param matImage The matrix of an image with double-precision data type
         */
        int RoundDoubleMatrix(cv::Mat &matImage);
        /**
          Calculate the FFT of image Image_mat and return the result in Image_FFT if sign equals to FFT_SIGN_TtoF. If sign equals to FFT_SIGN_FtoT, calculate the IFFT of Image_FFT and return the result in Image_mat.
          @pre Image_FFT must be non-null pointer.
          @param Image_mat The matrix of an image
          @param Image_FFT The pointer of an image's FFT
          @param sign A sign for designating FFT or IFFT calculation
          @param nonzerorows The expected rows of Image_FFT in FFT calculation or Image_mat in IFFT calculation. A appropriate rows can speed up the calculation.
         */
        int FFT_opencv(cv::Mat &Image_mat, IplImage *Image_FFT, int sign, int nonzerorows=0);

        /**
          Get the results of one channel.
          @param matC The calculated masked correlation of one channel
          @param matNumberOfOverMaskedPixels The number of overlap masked pixels in one channel.
          @param intChannel The channel number of the images.
         */
        double GetResult(cv::Mat &matC, cv::Mat &matNumberOfOverlapMaskedPixels, int intChannel);
};

}
#endif
#endif
