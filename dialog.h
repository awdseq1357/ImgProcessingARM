#ifndef DIALOG_H
#define DIALOG_H

//#define IMG_WIDTH 640
//#define IMG_HEIGHT 480

#include <QDialog>
#include <QImage>
#include <QPixmap>
#include <QString>
#include <QPainter>
#include <QImageReader>
#include "fytimgprocessinglib.h"
//#include "V4L2.h"

namespace Ui {
class Dialog;
}

class Dialog : public QDialog
{
    Q_OBJECT

public:
    explicit Dialog(QWidget *parent = 0);
    ~Dialog();

protected:
    void paintEvent(QPaintEvent* event);
   // void moveEvent(QMoveEvent* event);

private slots:
    void on_pushButton_exposure_compensation_clicked();

    void on_pushButton_read_image_clicked();

    void on_pushButton_test_image_clicked();

private:
    Ui::Dialog *ui;

    //wtret aaa;
    FYTImgProcessingLib processor;
    QImage image_;
    QPixmap pixmap_;
};

#endif // DIALOG_H
