#include "fytimgprocessinglib.h"

namespace
{
    unsigned int *rgbToGrayscale(QImage image);
    unsigned int *grayscaleToHistogram(unsigned int *grayscale_img);
    QImage *separateInBlocks(QImage image, int number_of_blocks);
}


FYTImgProcessingLib::FYTImgProcessingLib()
{
    //sobel开平方查找表赋值，输入值小于370*370
    table_sobel_sqrt = new unsigned char[136900];
    int i,j;
    for(i=0; i<255; i++)
    {
        int step_start = i*i;
        int step_end = (i+1)*(i+1);
        for(j=step_start; j<step_end; j++)
            table_sobel_sqrt[j] = i;
    }
    for(j=255*255; j<136900; j++)
        table_sobel_sqrt[j] = 255;
}

FYTImgProcessingLib::~FYTImgProcessingLib()
{
    delete table_sobel_sqrt;
}

QImage FYTImgProcessingLib::grayWorldWhiteBalance(QImage image)
{
    long long red_sum = 0, green_sum = 0, blue_sum = 0;

    for(int pixel_row = 0; pixel_row < image.height();pixel_row++)
    {
        QRgb *row_data = (QRgb*)image.scanLine(pixel_row);
        for(int pixel_col = 0; pixel_col < image.width(); pixel_col++)
        {
            QRgb pixel_data = row_data[pixel_col];
            red_sum += qRed(pixel_data);
            green_sum += qGreen(pixel_data);
            blue_sum += qBlue(pixel_data);
        }
    }

    float red_channel_gain = 0, blue_channel_gain = 0;
    if(red_sum!=green_sum || red_sum!=blue_sum || green_sum!=blue_sum)
    {
        red_channel_gain = (float)green_sum / (float)red_sum;
        blue_channel_gain = (float)green_sum / (float)blue_sum;


        for(int pixel_row = 0; pixel_row < image.height();pixel_row++)
        {
            QRgb *row_data = (QRgb*)image.scanLine(pixel_row);
            for(int pixel_col = 0; pixel_col < image.width(); pixel_col++)
            {
                QRgb *pixel_data = &row_data[pixel_col];
                int adjust_red_channel = red_channel_gain * qRed(*pixel_data);
                adjust_red_channel = adjust_red_channel <= 255? adjust_red_channel:255;
                int adjust_blue_channel = blue_channel_gain * qBlue(*pixel_data);
                adjust_blue_channel = adjust_blue_channel <= 255? adjust_blue_channel:255;
                *pixel_data = qRgb(adjust_red_channel,
                                   qGreen(*pixel_data),
                                   adjust_blue_channel);
            }
        }
    }
    return image;
}

QImage FYTImgProcessingLib::whitePitchWhiteBalance(QImage image)
{
    int red_max = 0, green_max = 0, blue_max = 0;
    for(int pixel_row = 0; pixel_row < image.height();pixel_row++)
    {
        QRgb *row_data = (QRgb*)image.scanLine(pixel_row);
        for(int pixel_col = 0; pixel_col < image.width(); pixel_col++)
        {
             QRgb pixel_data = row_data[pixel_col];
            if (red_max < qRed(pixel_data)) red_max = qRed(pixel_data);
            if (green_max < qGreen(pixel_data)) green_max = qGreen(pixel_data);
            if (blue_max < qBlue(pixel_data)) blue_max = qBlue(pixel_data);
        }
    }
    float red_channel_gain = (float)green_max / (float)red_max;
    float blue_channel_gain = (float)green_max / (float)blue_max;\

    //if the max_green channel is too small, scale the green channel up first
    int green_threshold = 20, adjust_factor = 1;
    if (green_max <= green_threshold)
        adjust_factor = 1.5;
    for(int pixel_row = 0; pixel_row < image.height();pixel_row++)
    {
        QRgb *row_data = (QRgb*)image.scanLine(pixel_row);
        for(int pixel_col = 0; pixel_col < image.width(); pixel_col++)
        {
            QRgb *pixel_data = &row_data[pixel_col];
            int adjust_red_channel = red_channel_gain * qRed(*pixel_data) * adjust_factor;
            adjust_red_channel = adjust_red_channel <= 255? adjust_red_channel:255;
            int adjust_blue_channel = blue_channel_gain * qBlue(*pixel_data) *adjust_factor;
            adjust_blue_channel = adjust_blue_channel <= 255? adjust_blue_channel:255;
            *pixel_data = qRgb(adjust_red_channel,
                               qGreen(*pixel_data) * adjust_factor,
                               adjust_blue_channel);
        }
    }
    return image;

}


QImage FYTImgProcessingLib::enhancedWhitePitchWhiteBalance(QImage image)
{
    long long red_sum = 0, red_square_sum = 0, red_max = 0, red_square_max = 0,
            green_sum = 0, green_max = 0,
            blue_sum = 0, blue_square_sum = 0, blue_max = 0, blue_square_max = 0;

    //save the value of square into a LUT
    long long *red_square_table = new long long [image.height() * image.width()]();
    long long *blue_square_table = new long long [image.height() * image.width()]();

    for(int pixel_row = 0; pixel_row < image.height();pixel_row++)
    {
        QRgb *row_data = (QRgb*)image.scanLine(pixel_row);
        for(int pixel_col = 0; pixel_col < image.width(); pixel_col++)
        {
            QRgb pixel_data = row_data[pixel_col];
            int current_pixel = pixel_row * image.width() + pixel_col;
            if (red_max < qRed(pixel_data)) red_max = qRed(pixel_data);
            if (green_max < qGreen(pixel_data)) green_max = qGreen(pixel_data);
            if (blue_max < qBlue(pixel_data)) blue_max = qBlue(pixel_data);

            red_sum += qRed(pixel_data);
            green_sum += qGreen(pixel_data);
            blue_sum += qBlue(pixel_data);

            red_square_table[current_pixel] = qRed(pixel_data)* qRed(pixel_data);
            blue_square_table[current_pixel] = qBlue(pixel_data) * qBlue(pixel_data);

            red_square_sum += red_square_table[current_pixel];
            blue_square_sum += blue_square_table[current_pixel];

            red_square_max = red_square_table[current_pixel] > red_square_max? red_square_table[current_pixel]
                                                                              : red_square_max;
            blue_square_max =  blue_square_table[current_pixel] > red_square_max? blue_square_table[current_pixel]
                                                                                  : blue_square_max;
        }
    }

    //denominator of the formula, product
    float de_factor_red_a = red_square_max * red_sum;
    float de_factor_red_b = red_square_sum * red_max;

    float red_modifier = (green_sum * red_square_max - green_max * red_square_sum)
            /(de_factor_red_a - de_factor_red_b);
    float red_square_modifier = (green_sum * red_max - green_max * red_sum)
            /(de_factor_red_b - de_factor_red_a);

    float de_factor_blue_a = blue_square_max * blue_sum;
    float de_factor_blue_b = blue_square_sum * blue_max;

    float blue_modifier = (green_sum * blue_square_max - green_max * blue_square_sum)
            /(de_factor_blue_a - de_factor_blue_b);
    float blue_square_modifier = (green_sum * blue_max - green_max * blue_sum)
            /(de_factor_blue_b - de_factor_blue_a);

    for(int pixel_row = 0; pixel_row < image.height();pixel_row++)
    {
        QRgb *row_data = (QRgb*)image.scanLine(pixel_row);
        for(int pixel_col = 0; pixel_col < image.width(); pixel_col++)
        {
            QRgb *pixel_data = &row_data[pixel_col];
            int current_pixel = pixel_row * image.width() + pixel_col;
            int adjust_red_channel = red_square_modifier * red_square_table[current_pixel]
                                    + red_modifier * qRed(*pixel_data);
            adjust_red_channel = adjust_red_channel <= 255? adjust_red_channel:255;
            int adjust_blue_channel = blue_square_modifier * blue_square_table[current_pixel]
                    + blue_modifier * qBlue(*pixel_data);
            adjust_blue_channel = adjust_blue_channel <= 255? adjust_blue_channel:255;
            *pixel_data = qRgb(adjust_red_channel,
                               qGreen(*pixel_data),
                               adjust_blue_channel);
        }
    }
    delete red_square_table;
    delete blue_square_table;
    return image;
}

int FYTImgProcessingLib::exposureCompensation(QImage test_image)
{
    //linear combination of contrast and focus measure;
    //S(selection) = F + D;
    int Laplacian_filter[] = {3,3,-1,-1,-1,-1,8,-1,-1,-1,-1};

    int selection = blockContrastMeasure(test_image) + blockFocusMeasure(test_image,Laplacian_filter);

    return selection;
}

QImage FYTImgProcessingLib::skinDetection(QImage image)
{
    //based on the color segmentation
    for(int pixel_row = 0; pixel_row < image.height();pixel_row++)
    {
        QRgb *row_data = (QRgb*)image.scanLine(pixel_row);
        for(int pixel_col = 0; pixel_col < image.width(); pixel_col++)
        {
            QRgb *pixel_data = &row_data[pixel_col];
            int red_threshold = 95, green_threshold = 40, blue_threshold = 20;
            int pixel_chrominance[] = {qRed(*pixel_data), qGreen(*pixel_data),qBlue(*pixel_data)};
            std::sort(pixel_chrominance,pixel_chrominance+3);
            bool is_skin = false;
            if (qRed(*pixel_data)> red_threshold
                    && qGreen(*pixel_data) > green_threshold
                    && qBlue(*pixel_data) > blue_threshold
                    && qRed(*pixel_data) > qGreen(*pixel_data)
                    && qRed(*pixel_data) > qBlue(*pixel_data))
            {
                if(pixel_chrominance[2] - pixel_chrominance[0] > 15)
                {
                    if(abs(qRed(*pixel_data) - qGreen(*pixel_data)))
                        is_skin = true;
                }
            }
            if(!is_skin) *pixel_data = qRgb(0,0,0);
            else *pixel_data = qRgb(255,255,255);
        }
    }
    return image;
}

QImage FYTImgProcessingLib::faceRecognition(QImage image)
{
    int num_face_candidate = 0;
    QImage segmented_image = skinDetection(image);
    ImageLocation *component_candidate = new ImageLocation;
    int *face_map = new int[image.width() * image.height()]();
    for(int pixel_row = 0; pixel_row < image.height();pixel_row++)
    {
        QRgb *row_data = (QRgb*)image.scanLine(pixel_row);
        for(int pixel_col = 0; pixel_col < image.width(); pixel_col++)
        {
            QRgb *pixel_data = &row_data[pixel_col];
            int current_pixel = pixel_row * image.width() + pixel_col;
            if(qRed(*pixel_data)!=0)
            {
                //FIXME: face recognition
                //face_map[current_pixel] = 1;
            }

        }
    }
    delete component_candidate;
    delete face_map;
    return image;
}

float FYTImgProcessingLib::blockContrastMeasure(
        QImage image
)
{
    unsigned int *histogram = grayscaleToHistogram(rgbToGrayscale(image));
    unsigned int *smooth_histogram = new unsigned int[256](); //FIXME:dynamic object
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
        smooth_histogram[scale] = histogram_component_smoothing_sum / 5;
        histogram_component_smoothing_sum = 0;

        //mean = sum(0-255) i * histogram(i) / sum(0-255) histogram(i)
        histogram_component_sum += smooth_histogram[scale];
        histogram_component_mean += scale * smooth_histogram[scale];
    }

    histogram_component_mean /= histogram_component_sum;

    float histogram_deviation = 0;
    for(int scale = 0; scale < 256; scale++)
    {
        histogram_deviation += ((scale - histogram_component_mean) * (scale- histogram_component_mean) * histogram[scale]);
    }
    histogram_deviation /= histogram_component_sum;

    if(0 <= histogram_deviation)
        return sqrt(histogram_deviation);
    else
        return -1;
}

//The output of a Laplacian filter is in the range of [-8 * 255, 8 * 255]
float FYTImgProcessingLib::blockFocusMeasure(QImage image, int *kernel)
{
    unsigned int *grayscale_img = rgbToGrayscale(image);
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
                        filtered_image[current_pixel + 2] += (grayscale_img[ii * img_width + jj + 2] * kernel[mm * kRows + nn + 2]);
                    }
                }
            }
            filtered_image[current_pixel + 2] =  filtered_image[current_pixel + 2] <=255?filtered_image[current_pixel + 2]:255;
            if (filtered_image[current_pixel + 2] >= kThreshold)
                focusMeasure+= filtered_image[current_pixel + 2];
        }
    }

    focusMeasure /= (img_width * img_height);
    delete filtered_image;
    return focusMeasure;
}

void FYTImgProcessingLib::sobelFilter(const unsigned char *img_8u1_src,
                                      unsigned char *img_8u1_mag,
                                      const int img_width,
                                      const int img_height)
{


    int i, j, offset, offset0;

    //最后一列的标号
    int last_width = img_width-1;
    //最后一行的标号
    int last_height = img_height-1;

    int Mx, My;

    //比图像宽度分布大于1和小于1的变量，避免重复计算时使用
    int widthL = img_width + 1;
    int widthS = img_width - 1;

    //边缘不处理
    offset = last_height*img_width;				//最后一行的起始位置
    for(i=0; i<img_width; i++)
    {
        img_8u1_mag[i] = 0;
        img_8u1_mag[offset+i] = 0;
    }
    for(j=0; j<img_height; j++)
    {
        offset = j*img_width;					//第j行的起点位置
        img_8u1_mag[offset] = 0;
        img_8u1_mag[offset+last_width] = 0;		//即第j行的最后一个位置
    }

    //对非边界的像素，求取其梯度响应值
    for(j=1; j<last_height; j++)
    {
        //第j行的偏移量
        offset0 = j*img_width;
        for(i=1; i<last_width; i++)
        {
            //第j行、第i列的偏移量
            offset = offset0 + i;

            //x方向分量
            Mx = img_8u1_src[offset+widthL] + ( img_8u1_src[offset+1] << 1 )+ img_8u1_src[offset-widthS];
            Mx -= img_8u1_src[offset+widthS] + ( img_8u1_src[offset-1] << 1 ) + img_8u1_src[offset-widthL];

            //y方向分量
            My = img_8u1_src[offset+widthL] + ( img_8u1_src[offset+img_width] << 1 ) + img_8u1_src[offset+widthS];
            My -= img_8u1_src[offset-widthS] + ( img_8u1_src[offset-img_width] << 1 ) + img_8u1_src[offset-widthL];

            //求取梯度幅度值
            img_8u1_mag[offset] = table_sobel_sqrt[(Mx*Mx+My*My)>>4];
        }
    }
}



void FYTImgProcessingLib::yuvToRgb(const unsigned char *img_8u2_yuyv,
                                   unsigned char *img_8u3_rgb,
                                   const int img_width,
                                   const int img_height)
{
    //YUV图像的尺寸
        int yuv_size = img_width * img_height * 2;

        //像素在图像数据中的偏移量
        int offset_rgb = 0;
        int offset_yuv = 0;

        //YUYV值
        int y0, u, y1, v;
        //RGB补偿值
        int delta_r, delta_g, delta_b;
        //RGB值
        int r, g, b;

        //将图像中的每一个YUYV值转化为2对RGB值
        for(offset_yuv = 0; offset_yuv < yuv_size; offset_yuv += 4)
        {
            //得到YUYV值
            y0 = img_8u2_yuyv[offset_yuv];
            u  = img_8u2_yuyv[offset_yuv + 1] - 128;		//128为色度的0位置
            y1 = img_8u2_yuyv[offset_yuv + 2];
            v  = img_8u2_yuyv[offset_yuv + 3] - 128;		//128为饱和度的0位置

            //得到UV的修正值
            delta_r = (351*v)>>8;
            delta_g = -( (179*v + 86*u)>>8 );
            delta_b = (444*u)>>8;

            //将Y0UV转化为RGB
            r = y0 + delta_r;
            g = y0 + delta_g;
            b = y0 + delta_b;
            if(r > 255) r = 255;
            if(g > 255) g = 255;
            if(b > 255) b = 255;
            if(r < 0) r = 0;
            if(g < 0) g = 0;
            if(b < 0) b = 0;
            img_8u3_rgb[offset_rgb++] = r;
            img_8u3_rgb[offset_rgb++] = g;
            img_8u3_rgb[offset_rgb++] = b;

            //将Y1UV转化为RGB
            r = y1 + delta_r;
            g = y1 + delta_g;
            b = y1 + delta_b;
            if(r > 255) r = 255;
            if(g > 255) g = 255;
            if(b > 255) b = 255;
            if(r < 0) r = 0;
            if(g < 0) g = 0;
            if(b < 0) b = 0;
            img_8u3_rgb[offset_rgb++] = r;
            img_8u3_rgb[offset_rgb++] = g;
            img_8u3_rgb[offset_rgb++] = b;
        }
}

void FYTImgProcessingLib::yuvToGrayscale(const unsigned char *img_8u2_yuyv,
                                         unsigned char *img_8u1_gray,
                                         const int img_width,
                                         const int img_height)
{
    int gray_size = img_width * img_height;
    int offset;
    for(offset = 0; offset < gray_size; offset ++)
    {
        img_8u1_gray[offset] = img_8u2_yuyv[offset<<1];
    }
}

void FYTImgProcessingLib::grayscaleToRgb(const unsigned char *img_8u1_gray,
                                         unsigned char *img_8u3_rgb,
                                         const int img_width,
                                         const int img_height)
{
    int offset_gray = 0;
    int offset_rgb = 0;
    int gray_size = img_width * img_height;

    for(offset_gray = 0; offset_gray < gray_size; offset_gray++)
    {
        img_8u3_rgb[offset_rgb++] = img_8u1_gray[offset_gray];
        img_8u3_rgb[offset_rgb++] = img_8u1_gray[offset_gray];
        img_8u3_rgb[offset_rgb++] = img_8u1_gray[offset_gray];
    }
}

QImage *FYTImgProcessingLib::separateInBlocks(QImage image, int dividend)
{
    int horizontal_step = image.width() / dividend;
    int vertical_step = image.height() / dividend;
    //TODO:dynamic array, clean
    QImage *separated_blocks = new QImage[dividend * dividend];
    for(int row_block = 0; row_block < dividend; row_block++)
    {
        for(int col_block = 0; col_block < dividend; col_block++)
        {
            separated_blocks[row_block * dividend + col_block] = image.copy(row_block * horizontal_step
                                                                            ,col_block* vertical_step
                                                                            ,horizontal_step
                                                                            ,vertical_step);
        }
    }
    return separated_blocks;
}

void FYTImgProcessingLib::testImageContrastFocus(QDir current_dir)
{
    QImageReader reader;
    int Laplacian_filter[] = {3,3,-1,-1,-1,-1,8,-1,-1,-1,-1};

    QImage test_image;
    QStringList image_filter;
    image_filter << "*.bmp" << "*.jpg" << "*.png" << "*.jpeg" <<"*.tif";
    QStringList image_name_list = current_dir.entryList(image_filter,QDir::Files|QDir::NoDotAndDotDot);
    QStringList::const_iterator image_name_iterator;
    for (image_name_iterator = image_name_list.constBegin(); image_name_iterator != image_name_list.constEnd();++image_name_iterator)
    {
        reader.setFileName(*image_name_iterator);
        qDebug() << *image_name_iterator;
        test_image = reader.read();
        if(*image_name_iterator == "solid_color_white.jpg")
        {
            float contrast = blockContrastMeasure(test_image);
            for(int pixel = 0; pixel < test_image.width() * test_image.height();pixel++)\
            {
              //  qDebug() << "pixel " + QString::number(pixel) << img[pixel];
            }

        }
//        testHistogram(*image_name_iterator);

        qDebug() << "contrast "<< blockContrastMeasure(test_image);
        qDebug() << "Focus "<< blockFocusMeasure(test_image, Laplacian_filter);
    }
}

void FYTImgProcessingLib::testHistogram(QString file_name)
{
    QImageReader reader;

    QImage test_image(file_name);
    unsigned int* histogram = grayscaleToHistogram(rgbToGrayscale(test_image));

    for(int i = 0; i< 256; i++)
        qDebug() << file_name + " histogram"<< "scale " + QString::number(i) <<histogram[i];
}

unsigned int *FYTImgProcessingLib::rgbToGrayscale(QImage image)
{
    int img_width = image.width(), img_height = image.height();
    //TODO: clean this object
    //assign the size of the array to position 0
    int img_array_length = img_width * img_height;
    unsigned int *grayscale_img = new unsigned int[img_width * img_height + 2]();
    grayscale_img[0] = img_width;
    grayscale_img[1] = img_height;

    for(int pixel_row = 0; pixel_row < img_height; pixel_row++)
    {
        QRgb *rowData = (QRgb*)image.scanLine(pixel_row);
        for(int pixel_col = 0; pixel_col < img_width; pixel_col++)
        {
            QRgb pixel_data = rowData[pixel_col];
            //TODO: call a function, could be slow
            grayscale_img[img_width * pixel_row + pixel_col + 2] = qGray(pixel_data);
        }
    }
    return grayscale_img;
}

unsigned int *FYTImgProcessingLib::grayscaleToHistogram(unsigned int *grayscale_img)
{
    //TODO clean this
    unsigned int *histogram = new unsigned int[256]();

    int count = 0;
    for(int pixel = 0; pixel < grayscale_img[0] * grayscale_img[1]; pixel++)
    {
        histogram[grayscale_img[pixel+2]]++;
    }
    return histogram;
}
