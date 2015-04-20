#include "dialog.h"
#include "ui_dialog.h"
#include <QDebug>
#include <QDir>

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
    //FYTImgProcessingLib processor;

    QImageReader reader;
    reader.setFileName("test.bmp");
    image_ = reader.read();
    //qDebug() << processor.blockContrastMeasure(image_);

    int Laplacin_filter[] = {3,3,-1,-1,-1,-1,8,-1,-1,-1,-1};
    processor.blockFocusMeasure(image_,Laplacin_filter);
    //qDebug() << image_.format() << image_.colorCount();
    pixmap_ = QPixmap::fromImage(image_);

    update();
}


