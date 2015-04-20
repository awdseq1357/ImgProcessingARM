#include "fytimgprocessinglib.h"

namespace
{
    unsigned int *rgbToGrayscale(QImage image);
    unsigned int *grayscaleToHistogram(unsigned int *grayscale_img);
    QImage *separateInBlocks(QImage image, int number_of_blocks);
}


FYTImgProcessingLib::FYTImgProcessingLib()
{
}

FYTImgProcessingLib::~FYTImgProcessingLib()
{

}

int FYTImgProcessingLib::exposureCompensation(QImage test_image)
{
    //linear combination of contrast and focus measure;
    //
    return 0;
}



float FYTImgProcessingLib::blockContrastMeasure(
        QImage image
)
{
    unsigned int *histogram = grayscaleToHistogram(rgbToGrayscale(image));

//    for(int i=0; i<255; i++)
//        qDebug()<< histogram[i];

    float histogram_component_smoothing_sum = 0;
    float histogram_component_sum = 0;
    float histogram_component_mean = 0;

    //smoothing the histogram
    for(int scale = 0; scale < 256; scale++)
    {
        if((scale-2) >= 0) histogram_component_smoothing_sum += histogram[scale-2];
        if((scale-1) >= 0) histogram_component_smoothing_sum += histogram[scale-1];
        histogram_component_smoothing_sum += histogram[scale];
        if((scale+1) <= 255) histogram_component_smoothing_sum += histogram[scale+1];
        if((scale+2) <= 255) histogram_component_smoothing_sum += histogram[scale+2];
        histogram[scale] = histogram_component_smoothing_sum / 5;
        histogram_component_smoothing_sum = 0;

        //mean = sum(0-255) i * histogram(i) / sum(0-255) histogram(i)
        histogram_component_sum += histogram[scale];
        histogram_component_mean = scale * histogram[scale];

    }

    histogram_component_mean /= histogram_component_sum;

    float histogram_deviation = 0;
    for(int scale = 0; scale < 256; scale++)
    {
        histogram_deviation += (scale - histogram_component_mean) * histogram[scale];
    }
    histogram_deviation /= histogram_component_sum;

    if(0 <= histogram_deviation)
        return histogram_deviation;
    else
        return -1;
}

//TODO: test for the return to be correct.
//The output of a Laplacian filter is in the range of [-8 * 255, 8 * 255]
float FYTImgProcessingLib::blockFocusMeasure(QImage image, int *kernel)
{
    //unsigned int *grayscale_img = rgbToGrayscale(image);
    unsigned int grayscale_img[] = {5,5,
                                  0,0,10,10,10,
                                  0,0,10,10,10,
                                  0,0,10,10,10,
                                  0,0,10,10,10,
                                  0,0,10,10,10,};


    //TODO: set the threshold
    int kThreshold = 0;
    //TODO: kernel was passed in as a pointer
    int kRows = kernel[0];
    int kCols = kernel[1];
    int kCenterX = kCols / 2;
    int kCenterY = kRows / 2;
    int img_width = grayscale_img[0], img_height = grayscale_img[1];
    //FIXME: same declaration as in another function
    //possible speed-up
    int *filtered_image = new int[img_width * img_height + 2]();
    filtered_image[0] = img_width, filtered_image[1] = img_height;
    float focusMeasure = 0;

    for(int pixel_row =0; pixel_row < img_height; ++pixel_row)              // rows
    {
        for(int pixel_column = 0; pixel_column < img_width; ++pixel_column)          // columns
        {
            int current_pixel = pixel_row * img_width + pixel_column;
            for(int kernel_row = 0; kernel_row < kRows; ++kernel_row)     // kernel rows
            {
                int mm = kRows - 1 - kernel_row;      // row index of flipped kernel
                for(int kernel_col = 0; kernel_col < kCols; ++kernel_col) // kernel columns
                {
                    int nn = kCols - 1 - kernel_col;  // column index of flipped kernel

                    // index of input signal, used for checking boundary
                    int ii = pixel_row + (kernel_row - kCenterY);
                    int jj = pixel_column + (kernel_col - kCenterX);

                    // ignore input samples which are out of bound
                    if( ii >= 0 && ii < img_height && jj >= 0 && jj < img_width )
                    {
                        filtered_image[current_pixel + 2] += grayscale_img[ii * img_width + jj + 2] * kernel[mm * kRows + nn + 2];
                        if (filtered_image[current_pixel + 2] >= kThreshold)
                            focusMeasure+= filtered_image[current_pixel + 2];
                    }
                }
            }
            qDebug() << current_pixel << " "<<filtered_image[current_pixel + 2];
        }
    }

    focusMeasure /= img_width * img_height;
    return focusMeasure;
}

namespace
{
QImage *separateInBlocks(QImage image, int number_of_blocks)
{
    return NULL;
}

unsigned int *rgbToGrayscale(QImage image)
{
    int img_width = image.width(), img_height = image.height();
    //TODO: clean this object
    //assign the size of the array to position 0
    int img_array_length = img_width * img_height;
    unsigned int *grayscale_img = new unsigned int[img_width * img_height + 1];
    grayscale_img[0] = img_width;
    grayscale_img[1] = img_height;

    for(int pixel_row = 0; pixel_row < img_height; pixel_row++)
    {
        QRgb *rowData = (QRgb*)image.scanLine(pixel_row);
        for(int pixel_col = 0; pixel_col < img_width; pixel_col++)
        {
            QRgb pixelData = rowData[pixel_col];
            //TODO: floating number calculation, could be slow on the embedded system
            grayscale_img[img_width * pixel_row + pixel_col + 2] = 0.2989 * qRed(pixelData) + 0.5870 * qGreen(pixelData) + 0.1140 * qBlue(pixelData);
        }
    }
    return grayscale_img;
}

unsigned int *grayscaleToHistogram(unsigned int *grayscale_img)
{
    //TODO: vector slower than the array
    std::vector<unsigned int> gray(grayscale_img[0] * grayscale_img[1],0);
    for(int i = 0; i < gray.size();i++)
    {
        gray[i] = grayscale_img[i+2];
    }

    //TODO clean this
    unsigned int *histogram = new unsigned int[255]();


    for(int pixel = 0; pixel < grayscale_img[0]; pixel++)
    {
        histogram[grayscale_img[pixel+2]]++;
    }
    return histogram;
}

}
