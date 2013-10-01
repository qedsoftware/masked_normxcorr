/***********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright 2013 The Math Path Inc. 
 * DBA: Quantitative Engineering Design (http://qe-design.com)
 * Authors: William Wu, Jiehua Chen, Zhang Zhiming
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

#include <math.h>
#include "normxcorr2_masked.hpp"

namespace cv {

//Xcorr_opencv constructor
Xcorr_opencv::Xcorr_opencv()
{
    eps=EPS/100.0;
}
//Xcorr_opencv destructor
Xcorr_opencv::~Xcorr_opencv()
{
    ;
}

//Initializing some member variables before calculation
int Xcorr_opencv::Initialization(string fixedImageName, string fixedMaskName, string movingImageName, string movingMaskName)
{
    cv::Mat tmpImage;

    //load images
    cv::Mat fixedImage = cv::imread(fixedImageName.c_str());
    cv::Mat fixedMask = cv::imread(fixedMaskName.c_str());
    cv::Mat movingImage = cv::imread(movingImageName.c_str());
    cv::Mat movingMask = cv::imread(movingMaskName.c_str());
    
    // print dimensions
    cout << "Fixed image (scene): " << fixedImageName << ": " << fixedImage.size() << endl;
    cout << "Fixed mask (scene mask): " << fixedMaskName << ": " << fixedMask.size() << endl;
    cout << "Moving image (template): " << movingImageName << ": " << movingImage.size() << endl;
    cout << "Moving mask (template mask): " <<  movingMaskName << ": " << movingMask.size() << endl;

    channelnum = fixedImage.channels();

    // Ensure that the masks consist of only 0s and 1s.  
    // Anything <= 0 is set to 0, and everything else is set to 1.
    threshold(fixedMask,fixedMask,0,1,CV_THRESH_BINARY);
    threshold(movingMask,movingMask,0,1,CV_THRESH_BINARY);

    // The fixed and moving images need to be masked for the equations below to
    // work correctly.
    fixedImage.copyTo(tmpImage,fixedMask);
    fixedImage = tmpImage.clone();
    movingImage.copyTo(tmpImage,movingMask);
    movingImage = tmpImage.clone();

    // Flip the moving image and mask in both dimensions so that its correlation
    // can be more easily handled.
    cv::Mat t,f;
    transpose(movingImage,t);
    flip(t,movingImage,1); 
    transpose(movingMask,t);
    flip(t,movingMask,1); 
    
    transpose(movingImage,t);
    flip(t,movingImage,1); 
    transpose(movingMask,t);
    flip(t,movingMask,1); 

    // Compute optimal FFT size
    // cvGetOptimalDFTSize returns minimum N >= size such that N = 2^p x 3^q x 5^r
    // for some p, q, r, which enables fast computation.
    combinedSize[0] = fixedImage.rows + movingImage.rows - 1;
    combinedSize[1] = fixedImage.cols + movingImage.cols - 1;

    optimalSize[0] = cvGetOptimalDFTSize(combinedSize[0]);
    optimalSize[1] = cvGetOptimalDFTSize(combinedSize[1]);
    // optimalSize[0] = combinedSize[0];
    // optimalSize[1] = combinedSize[1];
    optimalCvsize = cvSize(optimalSize[1], optimalSize[0]);
    
    fnorm = double(1) * double(optimalSize[0]) * double(optimalSize[1]) / 2.0;

    cout << "Dimensions of combined image: " << combinedSize[0] <<" x " << combinedSize[1] << endl;
    cout << "Optimal larger dimensions for fast DFT: " << optimalSize[0] <<" x " << optimalSize[1] << endl;

    // split image into separate channel images
    sbgr_fixedImage.resize(channelnum);
    sbgr_fixedMask.resize(channelnum);
    sbgr_movingImage.resize(channelnum);
    sbgr_movingMask.resize(channelnum);
    C_result.resize(channelnum);
    numberOfOverlapMaskedPixels_result.resize(channelnum);

    split(fixedImage,sbgr_fixedImage);
    split(fixedMask,sbgr_fixedMask);
    split(movingImage,sbgr_movingImage);
    split(movingMask,sbgr_movingMask);

    // initialize matrices 
    optFixedImage.create(optimalSize[0],optimalSize[1],CV_64FC1);
    optFixedImage_FFT = cvCreateImage(optimalCvsize,IPL_DEPTH_64F,2);
    optFixedMask.create(optimalSize[0],optimalSize[1],CV_64FC1);
    optFixedMask_FFT = cvCreateImage(optimalCvsize,IPL_DEPTH_64F,2);
    optMovingImage.create(optimalSize[0],optimalSize[1],CV_64FC1);
    optMovingImage_FFT = cvCreateImage(optimalCvsize,IPL_DEPTH_64F,2);
    optMovingMask.create(optimalSize[0],optimalSize[1],CV_64FC1);
    optMovingMask_FFT = cvCreateImage(optimalCvsize,IPL_DEPTH_64F,2);
    optFixedSquared.create(optimalSize[0],optimalSize[1],CV_64FC1);
    optFixedSquared_FFT = cvCreateImage(optimalCvsize,IPL_DEPTH_64F,2);
    optMovingSquared.create(optimalSize[0],optimalSize[1],CV_64FC1);
    optMovingSquared_FFT = cvCreateImage(optimalCvsize,IPL_DEPTH_64F,2);

    /*
    // display images
    std::string windowName = "fixedImage";
    cv::namedWindow( windowName, CV_WINDOW_AUTOSIZE); 
    cv::imshow( windowName, fixedImage ); 
    cv::waitKey(); 
    cv::imshow( windowName, fixedMask*255 ); 
    cv::waitKey(); 
    cv::imshow( windowName, movingImage ); 
    cv::waitKey(); 
    cv::imshow( windowName, movingMask*255 );
    cv::waitKey(); 
    */

    return 0;
}

//Calculate the masked correlations of all channels.
int Xcorr_opencv::CalXcorr()
{
    
    for(int i = 0; i < channelnum; i++)
    {
        optFixedImage = Mat::zeros(optimalSize[0],optimalSize[1],CV_64FC1);
        optFixedMask = Mat::zeros(optimalSize[0],optimalSize[1],CV_64FC1);
        optMovingImage = Mat::zeros(optimalSize[0],optimalSize[1],CV_64FC1);
        optMovingMask = Mat::zeros(optimalSize[0],optimalSize[1],CV_64FC1);
        optFixedSquared = Mat::zeros(optimalSize[0],optimalSize[1],CV_64FC1);
        optMovingSquared = Mat::zeros(optimalSize[0],optimalSize[1],CV_64FC1);
        for(int j = 0; j < sbgr_fixedImage[i].rows; j++)
        {
            for(int k = 0; k < sbgr_fixedImage[i].cols; k++)
            {
                optFixedImage.at<double>(j,k) = sbgr_fixedImage[i].at<unsigned char>(j,k);
            }
        }
        for(int j = 0; j < sbgr_fixedMask[i].rows; j++)
        {
            for(int k = 0; k < sbgr_fixedMask[i].cols; k++)
            {
                optFixedMask.at<double>(j,k) = sbgr_fixedMask[i].at<unsigned char>(j,k);
            }
        }
        for(int j = 0; j < sbgr_movingImage[i].rows; j++)
        {
            for(int k = 0; k < sbgr_movingImage[i].cols; k++)
            {
                optMovingImage.at<double>(j,k) = sbgr_movingImage[i].at<unsigned char>(j,k);
            }
        }
        for(int j = 0; j < sbgr_movingMask[i].rows; j++)
        {
            for(int k = 0; k < sbgr_movingMask[i].cols; k++)
            {
                optMovingMask.at<double>(j,k) = sbgr_movingMask[i].at<unsigned char>(j,k);
            }
        }
        double t = (double)getTickCount();
        CalculateOneChannelXcorr(i);
        t = (double)getTickCount() - t; 
        printf("\tCalculated cross-correlation for one channel in %g ms\n", t*1000/getTickFrequency());
    }

    return 0;
}

//Calculate the masked correlation of one channel.
int Xcorr_opencv::CalculateOneChannelXcorr(int curChannel)
{

    if((!optFixedImage.isContinuous()))
    {
        printf("error: not continuous\n");
        exit(1);
    }
    // Only 6 FFTs are needed.
    FFT_opencv( optFixedImage,optFixedImage_FFT,FFT_SIGN_TtoF, sbgr_fixedMask[0].rows);
    FFT_opencv( optMovingImage, optMovingImage_FFT,FFT_SIGN_TtoF, sbgr_movingImage[0].rows);
    FFT_opencv( optFixedMask, optFixedMask_FFT,FFT_SIGN_TtoF, sbgr_fixedMask[0].rows);
    FFT_opencv( optMovingMask, optMovingMask_FFT,FFT_SIGN_TtoF, sbgr_movingImage[0].rows);

    // Compute and save these results
    cv::Mat numberOfOverlapMaskedPixels;
    IplImage *numberOfOverlapMaskedPixels_FFT;
    numberOfOverlapMaskedPixels.create(optimalSize[0],optimalSize[1],CV_64FC1);
    numberOfOverlapMaskedPixels_FFT = cvCreateImage(optimalCvsize,IPL_DEPTH_64F,2);
    cvMulSpectrums(optMovingMask_FFT, optFixedMask_FFT, numberOfOverlapMaskedPixels_FFT, 0);
    FFT_opencv( numberOfOverlapMaskedPixels, numberOfOverlapMaskedPixels_FFT,FFT_SIGN_FtoT, optimalSize[0]);

    RoundDoubleMatrix(numberOfOverlapMaskedPixels);
    ThresholdLower(numberOfOverlapMaskedPixels, eps / 1000);

    cv::Mat maskCorrelatedFixed;
    IplImage *maskCorrelatedFixedFFT;
    maskCorrelatedFixed.create(optimalSize[0],optimalSize[1],CV_64FC1);
    maskCorrelatedFixedFFT = cvCreateImage(optimalCvsize,IPL_DEPTH_64F,2);
    cvMulSpectrums(optMovingMask_FFT,optFixedImage_FFT,maskCorrelatedFixedFFT,0);
    FFT_opencv( maskCorrelatedFixed, maskCorrelatedFixedFFT,FFT_SIGN_FtoT, optimalSize[0]);

    cv::Mat maskCorrelatedRotatedMoving;
    IplImage *maskCorrelatedRotatedMovingFFT;
    maskCorrelatedRotatedMoving.create(optimalSize[0],optimalSize[1],CV_64FC1);
    maskCorrelatedRotatedMovingFFT = cvCreateImage(optimalCvsize,IPL_DEPTH_64F,2);
    cvMulSpectrums(optFixedMask_FFT,optMovingImage_FFT,maskCorrelatedRotatedMovingFFT,0);
    FFT_opencv( maskCorrelatedRotatedMoving, maskCorrelatedRotatedMovingFFT,FFT_SIGN_FtoT, optimalSize[0]);

    cv::Mat numerator;
    IplImage *numerator_FFT;
    numerator.create(optimalSize[0],optimalSize[1],CV_64FC1);
    numerator_FFT = cvCreateImage(optimalCvsize,IPL_DEPTH_64F,2);
    cvMulSpectrums(optMovingImage_FFT,optFixedImage_FFT,numerator_FFT,0);
    FFT_opencv( numerator, numerator_FFT,FFT_SIGN_FtoT, optimalSize[0]);

    numerator = numerator -( (maskCorrelatedFixed.mul(maskCorrelatedRotatedMoving)) / numberOfOverlapMaskedPixels);

    optFixedSquared = optFixedImage.mul(optFixedImage);
    FFT_opencv( optFixedSquared, optFixedSquared_FFT,FFT_SIGN_TtoF, sbgr_fixedImage[0].rows);

    cv::Mat fixedDenom;
    IplImage *fixedDenom_FFT;
    fixedDenom.create(optimalSize[0],optimalSize[1],CV_64FC1);
    fixedDenom_FFT = cvCreateImage(optimalCvsize,IPL_DEPTH_64F,2);
    cvMulSpectrums(optMovingMask_FFT,optFixedSquared_FFT,fixedDenom_FFT,0);
    FFT_opencv( fixedDenom, fixedDenom_FFT,FFT_SIGN_FtoT, optimalSize[0]);

    fixedDenom = fixedDenom - ((maskCorrelatedFixed.mul(maskCorrelatedFixed)) / numberOfOverlapMaskedPixels);
    ThresholdLower(fixedDenom,0);

    optMovingSquared = optMovingImage.mul(optMovingImage);
    FFT_opencv( optMovingSquared, optMovingSquared_FFT,FFT_SIGN_TtoF, sbgr_movingImage[0].rows);

    cv::Mat movingDenom;
    IplImage *movingDenom_FFT;
    movingDenom.create(optimalSize[0],optimalSize[1],CV_64FC1);
    movingDenom_FFT = cvCreateImage(optimalCvsize,IPL_DEPTH_64F,2);
    cvMulSpectrums(optFixedMask_FFT,optMovingSquared_FFT,movingDenom_FFT,0);
    FFT_opencv( movingDenom, movingDenom_FFT,FFT_SIGN_FtoT, optimalSize[0]);

    movingDenom = movingDenom - ((maskCorrelatedRotatedMoving.mul(maskCorrelatedRotatedMoving)) / numberOfOverlapMaskedPixels);
    ThresholdLower(movingDenom,0);

    cv::Mat denom = fixedDenom.mul(movingDenom);
    sqrt(denom,denom);

    // denom is the sqrt of the product of positive numbers so it must be
    // positive or zero.  Therefore, the only danger in dividing the numerator
    // by the denominator is when dividing by zero.  
    // Since the correlation value must be between -1 and 1, we therefore
    // saturate at these values.
    cv::Mat C = Mat::zeros(numerator.rows,numerator.cols,CV_64FC1);
    double maxAbs = MaxAbsValue(denom);
    double tol = 1000 * eps * maxAbs;
    double movingMaskSize = MatrixSum(optMovingMask);
    PostProcessing(C, numerator, denom, tol, -1, 1, numberOfOverlapMaskedPixels, movingMaskSize); 

    // Crop out the correct size.
    C_result[curChannel] = C(Range(0,combinedSize[0]),Range(0,combinedSize[1]));
    numberOfOverlapMaskedPixels_result[curChannel] = numberOfOverlapMaskedPixels(Range(0,combinedSize[0]),Range(0,combinedSize[1]));
    return 0;
}

// Divides numerator matrix by denominator matrix elementwise, scales results to 
// [-1,1], and discards points with small overlap: 
//  if (overlap < templateMask.size): set correlation to 0
int Xcorr_opencv::PostProcessing(cv::Mat &matC, cv::Mat &matNumerator, cv::Mat &matDenom, double tol, double minimum, double maximum, cv::Mat &numberOfOverlapMaskedPixels, double minimumOverlapSize)
{
    for(int i = 0; i < matDenom.rows;i++)
    {
        for(int j = 0; j < matDenom.cols;j++)
        {
            if (numberOfOverlapMaskedPixels.at<double>(i,j) < minimumOverlapSize) {
                matC.at<double>(i,j) = 0;
            } else {
                if(std::abs(matDenom.at<double>(i,j)) > tol)
                {
                    matC.at<double>(i,j) = matNumerator.at<double>(i,j) / matDenom.at<double>(i,j);
                }
                if(matC.at<double>(i,j) > maximum)
                {
                    matC.at<double>(i,j) = maximum;
                }
                if(matC.at<double>(i,j) < minimum)
                {
                    matC.at<double>(i,j) = minimum;
                }
            }
        }
    }
    return 0;
}

// Divide matNumerator by matDenom elementwise when the corresponding element of matDenom is nonzero, 
// Otherwise, the corresponding element of matNumerator is unchanged.
int Xcorr_opencv::DivideNonZeroElem(cv::Mat &matC, cv::Mat &matNumerator, cv::Mat &matDenom, double tol)
{
    for(int i = 0; i < matDenom.rows;i++)
    {
        for(int j = 0; j < matDenom.cols;j++)
        {
            if(std::abs(matDenom.at<double>(i,j)) > tol)
            {
                matC.at<double>(i,j) = matNumerator.at<double>(i,j) / matDenom.at<double>(i,j);
            }
        }
    }
    return 0;
}

// Return sum of all values in the matrix
double Xcorr_opencv::MatrixSum(cv::Mat &matImage)
{
    double sum = 0;
    for(int i = 0; i < matImage.rows; i++)
    {
        for(int j = 0; j < matImage.cols; j++)
        {
            sum += matImage.at<double>(i,j);
        }
    }
    return sum;
}

// Return maximum absolute value of all elements in matrix.
double Xcorr_opencv::MaxAbsValue(cv::Mat &matImage )
{
    double minVal, maxVal;
    minMaxLoc(matImage, &minVal, &maxVal, NULL, NULL, noArray());
    return std::max(std::abs(minVal),std::abs(maxVal));
}

//Calculate the FFT of image Image_mat and return the result in Image_FFT 
//if sign equals to FFT_SIGN_TtoF. If sign equals to FFT_SIGN_FtoT, 
//calculate the IFFT of Image_FFT and return the result in Image_mat.
int Xcorr_opencv::FFT_opencv(cv::Mat &Image_mat, IplImage *Image_FFT, int sign, int nonzerorows)
{
    if(sign == FFT_SIGN_TtoF)
    {
        IplImage *dst = Image_FFT;
        IplImage Image_Ipl = IplImage(Image_mat);
        IplImage *src = &Image_Ipl;
        {   
             IplImage *image_Re = 0, *image_Im = 0, *Fourier = 0;
             image_Re = cvCreateImage(cvGetSize(src), IPL_DEPTH_64F, 1);  
             //Imaginary part
             image_Im = cvCreateImage(cvGetSize(src), IPL_DEPTH_64F, 1);  
             //2 channels (image_Re, image_Im)
             Fourier = cvCreateImage(cvGetSize(src), IPL_DEPTH_64F, 2);
             // Real part conversion from u8 to 64f (double)
             cvConvertScale(src, image_Re, 1, 0);
             // Imaginary part (zeros)
             cvZero(image_Im);
             // Join real and imaginary parts and stock them in Fourier image
             cvMerge(image_Re, image_Im, 0, 0, Fourier);
             // Application of the forward Fourier transform
             cvDFT(Fourier, dst, CV_DXT_FORWARD, nonzerorows);
             cvReleaseImage(&image_Re);
             cvReleaseImage(&image_Im);
             cvReleaseImage(&Fourier);
        }
    }
    else
    {
        IplImage *ImageRe;
        IplImage *ImageIm;
        IplImage *dst;
        IplImage Image_Ipl = IplImage(Image_mat);
        ImageRe = &Image_Ipl;
        ImageIm = cvCreateImage(cvGetSize(ImageRe),IPL_DEPTH_64F,1);
        dst = cvCreateImage(cvGetSize(ImageRe),IPL_DEPTH_64F,2);
        cvDFT(Image_FFT,dst,CV_DXT_INV_SCALE, nonzerorows);
        cvSplit(dst,ImageRe,ImageIm,0,0);
        cvReleaseImage(&ImageIm);
        cvReleaseImage(&dst);
    }
    return 0;
}

//Scan matrix and compare each element with minimum. 
//Assign all values less than minimum to minimum.
int Xcorr_opencv::ThresholdLower(cv::Mat &matImage, double minimum)
{
    for(int i = 0; i < matImage.rows; i++)
    {
        for(int j = 0; j < matImage.cols; j++)
        {
            if(matImage.at<double>(i,j) < minimum)
            {
                matImage.at<double>(i,j) = minimum;
            }
        }
    }
    return 0;
}

//Scan matrix and compare each element with maximum. 
//Assign all values larger than maximum to maximum.
int Xcorr_opencv::ThresholdUpper(cv::Mat &matImage, double maximum)
{
    for(int i =0; i < matImage.rows; i++)
    {
        for(int j = 0; j < matImage.cols; j++)
        {
            if(matImage.at<double>(i,j) > maximum)
            {
                matImage.at<double>(i,j) = maximum;
            }
        }
    }
    return 0;
}

//Call round function on each element of the matrix.
int Xcorr_opencv::RoundDoubleMatrix(cv::Mat &matImage)
{
    for(int i =0;i < matImage.rows;i++)
    {
        for(int j = 0;j < matImage.cols;j++)
        {
            matImage.at<double>(i,j) = round(matImage.at<double>(i,j));
        }
    }
    return 0;
}

//Get results for one channel.
double Xcorr_opencv::GetResult(cv::Mat &matC, cv::Mat &matNumberOfOverlapMaskedPixels, int intChannel)
{
    if(intChannel >= channelnum || intChannel < 0)
    {
        return -1;
    }
    matC = C_result[intChannel].clone();
    matNumberOfOverlapMaskedPixels = numberOfOverlapMaskedPixels_result[intChannel].clone();
    return 0;
}

}

