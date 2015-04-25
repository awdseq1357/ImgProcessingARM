#include "dialog.h"
#include "ui_dialog.h"
#include <QDebug>
#include <QDir>
namespace
{
    void testFunction(QString file_name)
    {

    }
}
Dialog::Dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialog)
{
    ui->setupUi(this);

    image_ = QImage();
    processor = FYTImgProcessingLib();
}

Dialog::~Dialog()
{
    delete ui;
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



    //int Laplacin_filter[] = {3,3,-1,-1,-1,-1,8,-1,-1,-1,-1};
    int IMAGE_WIDTH = 640, IMAGE_HEIGHT = 480;
    //TODO: dynamic object
    //unsigned char *m_img_8u1_mag = new unsigned char[IMAGE_WIDTH*IMAGE_HEIGHT];
    //QImage m_image = QImage(IMAGE_WIDTH, IMAGE_HEIGHT, QImage::Format_RGB888);
    //qDebug() << image_.format() << image_.colorCount();
    //processor.sobelFilter(processor.rgbToGrayscale(image_),m_img_8u1_mag,IMAGE_WIDTH,IMAGE_HEIGHT);
    //processor.grayscaleToRgb(m_img_8u1_mag,m_image.bits(),IMAGE_WIDTH, IMAGE_HEIGHT);
    //image_ = processor.enhancedWhitePitchWhiteBalance(image_);
    //image_ = processor.skinDetection(image_);
    pixmap_ = QPixmap::fromImage(image_);

    update();
}



void Dialog::on_pushButton_test_image_clicked()
{
    processor.testImageContrastFocus(QDir::current());
}
