#include "dialog.h"
#include "ui_dialog.h"
#include <QDebug>
#include <QDir>
namespace
{
}
Dialog::Dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialog)
{
    ui->setupUi(this);

    image_ = QImage();
    processor = new FYTImgProcessingLib;
}

Dialog::~Dialog()
{
    delete ui;
    delete processor;
}

void Dialog::paintEvent(QPaintEvent *event)
{

    QPainter painter(this);
    painter.drawPixmap(10, 10, pixmap_);
    //QWidget::paintEvent(event);
}

void Dialog::on_pushButton_exposure_compensation_clicked()
{

}

void Dialog::on_pushButton_read_image_clicked()
{



    int IMAGE_WIDTH = 640, IMAGE_HEIGHT = 480;
;   processor->readImage("UE_2.jpg");
    //QImage m_image = QImage(IMAGE_WIDTH, IMAGE_HEIGHT, QImage::Format_RGB888);
    //qDebug() << image_.format() << image_.colorCount();
    //processor.sobelFilter(processor.rgbToGrayscale(image_),m_img_8u1_mag,IMAGE_WIDTH,IMAGE_HEIGHT);
    //processor.grayscaleToRgb(m_img_8u1_mag,m_image.bits(),IMAGE_WIDTH, IMAGE_HEIGHT);
    //image_ = processor.enhancedWhitePitchWhiteBalance(image_);
    //processor.relevantRegionSelection(8,false);
    QImage *blocks = processor->separated_blocks;
    image_ = processor->luminanceAdjust(processor->relevantRegionSelection(8),8);
    //image_ = blocks[0];
    //image_ = processor->enhancedWhitePitchWhiteBalance(image_);
    image_ = image_.scaledToWidth(640);
    pixmap_ = QPixmap::fromImage(image_);

    update();
}



void Dialog::on_pushButton_test_image_clicked()
{
    processor->testImageContrastFocus(QDir::current());
}
