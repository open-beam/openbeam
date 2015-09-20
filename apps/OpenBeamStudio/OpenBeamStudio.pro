#-------------------------------------------------
#
# Project created by QtCreator 2015-06-28T12:28:36
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = OpenBeamStudio
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
		highlighter.cpp 

HEADERS  += mainwindow.h \
		highlighter.h

FORMS    += mainwindow.ui

RESOURCES += \
    openbeamstudio.qrc
