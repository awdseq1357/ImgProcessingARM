#ifndef FYTIMGPROCESSINGLIB_H
#define FYTIMGPROCESSINGLIB_H

#include <QImage>
#include <QRgb>
#include <vector>
#include <QDebug>
#include <QDir>
#include <QStringList>
#include <QImageReader>
#include <algorithm>
#include <cmath>

#define IMG_WIDTH 640
#define IMG_HEIGHT 480

class FYTImgProcessingLib
{
    struct ImageLocation
    {
        int up_left;
        int up_right;
        int down_left;
        int down_right;
    };

public:
    FYTImgProcessingLib();
    ~FYTImgProcessingLib();

    QImage grayWorldWhiteBalance(QImage image);
    QImage whitePitchWhiteBalance(QImage image);
    QImage enhancedWhitePitchWhiteBalance(QImage image);

    int exposureCompensation(QImage test_image);
    int exposureCompensation(unsigned int camera_image);

    //TODO: skin detection
    QImage skinDetection(QImage image);
    //TODO: face recognition
    QImage faceRecognition(QImage image);
    //return -1 == error
    float blockContrastMeasure(QImage image);

    //with Laplacian filter
    // 1 1 1
    // 1 -8 1
    // 1 1 1
    //kernel: position 0 == number of rows
    //        position 1 == number of columns
    float blockFocusMeasure(QImage image, int *kernel);


    void sobelFilter(const unsigned char *img_8u1_src,
            unsigned char *img_8u1_mag,
            const int img_width,
            const int img_height);

    //Data format Conversion
    void yuvToRgb(const unsigned char *img_8u2_yuyv,
                  unsigned char *img_8u3_rgb,
                  const int img_width,
                  const int img_height);

    void yuvToGrayscale(const unsigned char *img_8u2_yuyv,
                        unsigned char *img_8u1_gray,
                        const int img_width,
                        const int img_height);

    void grayscaleToRgb(const unsigned char *img_8u1_gray, unsigned char *img_8u3_rgb, const int img_width, const int img_height);
    unsigned int *rgbToGrayscale(QImage image);
    unsigned int *grayscaleToHistogram(unsigned int *grayscale_img);

    //number_of_blocks == divident * dividend
    QImage *separateInBlocks(QImage image, int dividend);

    int cameraResponse(int light_intensity);
    void testImageContrastFocus(QDir current_dir);
    void testHistogram(QString file_name);
private:
    int image;
    QImage input_image;
    QImage visibility_image;

    int camera_response_curve_LUT[255];
    unsigned char table_sobel_sqrt[136900];


};

#endif // FYTIMGPROCESSINGLIB_H
