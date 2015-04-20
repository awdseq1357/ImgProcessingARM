#ifndef FYTIMGPROCESSINGLIB_H
#define FYTIMGPROCESSINGLIB_H

#include <QImage>
#include <QRgb>
#include <vector>
#include <iostream>
#include <QDebug>

#define IMG_WIDTH 640
#define IMG_HEIGHT 480

class FYTImgProcessingLib
{
public:
    FYTImgProcessingLib();
    ~FYTImgProcessingLib();

    //TODO: white balance
//    int whiteBalance();

    int exposureCompensation(QImage test_image);
    int exposureCompensation(unsigned int camera_image);


    //TODO: skin detection

    //return -1 == error
    float blockContrastMeasure(QImage image);

    //with Laplacian filter
    // 1 1 1
    // 1 -8 1
    // 1 1 1
    //kernel: position 0 == number of rows
    //        position 1 == number of columns
    float blockFocusMeasure(QImage image, int *kernel);


    //TODO: sobel operator: edge detection
//    int sobelFilter();
private:
    int image;

    QImage input_image;
    QImage visibility_image;
};

#endif // FYTIMGPROCESSINGLIB_H
