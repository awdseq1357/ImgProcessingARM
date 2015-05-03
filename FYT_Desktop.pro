#-------------------------------------------------
#
# Project created by QtCreator 2015-04-18T10:33:21
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = FYT_Desktop
TEMPLATE = app


SOURCES += main.cpp\
        dialog.cpp \
    fytimgprocessinglib.cpp \
    qsLib/avilib.c \
    V4L2/V4L2.cpp

HEADERS  += dialog.h \
    fytimgprocessinglib.h \
    qsLib/avilib.h \
    qsLib/qsImgLib.h \
    V4L2/V4L2.h

FORMS    += dialog.ui

DISTFILES += \
    libqsImgLib.so \
    libqsImgLib.so.1 \
    libqsImgLib.so.1.0 \
    libqsImgLib.so.1.0.0
