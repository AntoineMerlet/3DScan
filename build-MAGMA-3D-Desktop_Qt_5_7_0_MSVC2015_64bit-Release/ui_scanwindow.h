/********************************************************************************
** Form generated from reading UI file 'scanwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.7.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SCANWINDOW_H
#define UI_SCANWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>
#include "QVTKWidget.h"

QT_BEGIN_NAMESPACE

class Ui_scanwindow
{
public:
    QWidget *centralwidget;
    QLabel *sw_scanparameters_label;
    QFrame *sw_parameters_frame;
    QLabel *sw_xmin_label;
    QLabel *sw_xmax_label;
    QLabel *sw_ymin_label;
    QLabel *sw_ymax_label;
    QLabel *sw_zmin_label;
    QLabel *sw_zmax_label;
    QSlider *sw_xmin_horslider;
    QSlider *sw_xmax_horslider;
    QSlider *sw_ymin_horslider;
    QSlider *sw_ymax_horslider;
    QSlider *sw_zmin_horslider;
    QSlider *sw_zmax_horslider;
    QCheckBox *sw_filtering_checkbox;
    QPushButton *sw_startscan_pushbutton;
    QPushButton *sw_stopscan_pushbutton;
    QLabel *sw_visualizer_label;
    QPushButton *sw_capturepc_puchbutton;
    QLabel *sw_pcnumber_label;
    QLineEdit *sw_pcnumber_lineedit;
    QRadioButton *sw_horizontalacq_radiobutton;
    QRadioButton *sw_verticalacq_radiobutton;
    QLabel *sw_acqtype_label;
    QVTKWidget *sw_qvtkWidget;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *scanwindow)
    {
        if (scanwindow->objectName().isEmpty())
            scanwindow->setObjectName(QStringLiteral("scanwindow"));
        scanwindow->resize(700, 550);
        centralwidget = new QWidget(scanwindow);
        centralwidget->setObjectName(QStringLiteral("centralwidget"));
        sw_scanparameters_label = new QLabel(centralwidget);
        sw_scanparameters_label->setObjectName(QStringLiteral("sw_scanparameters_label"));
        sw_scanparameters_label->setGeometry(QRect(20, 60, 81, 16));
        sw_parameters_frame = new QFrame(centralwidget);
        sw_parameters_frame->setObjectName(QStringLiteral("sw_parameters_frame"));
        sw_parameters_frame->setGeometry(QRect(20, 80, 181, 261));
        sw_parameters_frame->setFrameShape(QFrame::Box);
        sw_parameters_frame->setFrameShadow(QFrame::Raised);
        sw_xmin_label = new QLabel(sw_parameters_frame);
        sw_xmin_label->setObjectName(QStringLiteral("sw_xmin_label"));
        sw_xmin_label->setGeometry(QRect(20, 10, 61, 16));
        sw_xmax_label = new QLabel(sw_parameters_frame);
        sw_xmax_label->setObjectName(QStringLiteral("sw_xmax_label"));
        sw_xmax_label->setGeometry(QRect(20, 50, 61, 16));
        sw_ymin_label = new QLabel(sw_parameters_frame);
        sw_ymin_label->setObjectName(QStringLiteral("sw_ymin_label"));
        sw_ymin_label->setGeometry(QRect(20, 90, 51, 16));
        sw_ymax_label = new QLabel(sw_parameters_frame);
        sw_ymax_label->setObjectName(QStringLiteral("sw_ymax_label"));
        sw_ymax_label->setGeometry(QRect(20, 130, 61, 16));
        sw_zmin_label = new QLabel(sw_parameters_frame);
        sw_zmin_label->setObjectName(QStringLiteral("sw_zmin_label"));
        sw_zmin_label->setGeometry(QRect(20, 170, 51, 16));
        sw_zmax_label = new QLabel(sw_parameters_frame);
        sw_zmax_label->setObjectName(QStringLiteral("sw_zmax_label"));
        sw_zmax_label->setGeometry(QRect(20, 210, 61, 16));
        sw_xmin_horslider = new QSlider(sw_parameters_frame);
        sw_xmin_horslider->setObjectName(QStringLiteral("sw_xmin_horslider"));
        sw_xmin_horslider->setGeometry(QRect(10, 30, 160, 19));
        sw_xmin_horslider->setOrientation(Qt::Horizontal);
        sw_xmax_horslider = new QSlider(sw_parameters_frame);
        sw_xmax_horslider->setObjectName(QStringLiteral("sw_xmax_horslider"));
        sw_xmax_horslider->setGeometry(QRect(10, 70, 160, 19));
        sw_xmax_horslider->setOrientation(Qt::Horizontal);
        sw_ymin_horslider = new QSlider(sw_parameters_frame);
        sw_ymin_horslider->setObjectName(QStringLiteral("sw_ymin_horslider"));
        sw_ymin_horslider->setGeometry(QRect(10, 110, 160, 19));
        sw_ymin_horslider->setOrientation(Qt::Horizontal);
        sw_ymax_horslider = new QSlider(sw_parameters_frame);
        sw_ymax_horslider->setObjectName(QStringLiteral("sw_ymax_horslider"));
        sw_ymax_horslider->setGeometry(QRect(10, 150, 160, 19));
        sw_ymax_horslider->setOrientation(Qt::Horizontal);
        sw_zmin_horslider = new QSlider(sw_parameters_frame);
        sw_zmin_horslider->setObjectName(QStringLiteral("sw_zmin_horslider"));
        sw_zmin_horslider->setGeometry(QRect(10, 190, 160, 19));
        sw_zmin_horslider->setOrientation(Qt::Horizontal);
        sw_zmax_horslider = new QSlider(sw_parameters_frame);
        sw_zmax_horslider->setObjectName(QStringLiteral("sw_zmax_horslider"));
        sw_zmax_horslider->setGeometry(QRect(10, 230, 160, 19));
        sw_zmax_horslider->setOrientation(Qt::Horizontal);
        sw_filtering_checkbox = new QCheckBox(centralwidget);
        sw_filtering_checkbox->setObjectName(QStringLiteral("sw_filtering_checkbox"));
        sw_filtering_checkbox->setGeometry(QRect(30, 360, 121, 17));
        sw_startscan_pushbutton = new QPushButton(centralwidget);
        sw_startscan_pushbutton->setObjectName(QStringLiteral("sw_startscan_pushbutton"));
        sw_startscan_pushbutton->setGeometry(QRect(30, 480, 75, 23));
        sw_stopscan_pushbutton = new QPushButton(centralwidget);
        sw_stopscan_pushbutton->setObjectName(QStringLiteral("sw_stopscan_pushbutton"));
        sw_stopscan_pushbutton->setGeometry(QRect(130, 480, 75, 23));
        sw_visualizer_label = new QLabel(centralwidget);
        sw_visualizer_label->setObjectName(QStringLiteral("sw_visualizer_label"));
        sw_visualizer_label->setGeometry(QRect(220, 10, 47, 13));
        sw_capturepc_puchbutton = new QPushButton(centralwidget);
        sw_capturepc_puchbutton->setObjectName(QStringLiteral("sw_capturepc_puchbutton"));
        sw_capturepc_puchbutton->setGeometry(QRect(30, 390, 171, 23));
        sw_pcnumber_label = new QLabel(centralwidget);
        sw_pcnumber_label->setObjectName(QStringLiteral("sw_pcnumber_label"));
        sw_pcnumber_label->setGeometry(QRect(30, 450, 121, 16));
        sw_pcnumber_lineedit = new QLineEdit(centralwidget);
        sw_pcnumber_lineedit->setObjectName(QStringLiteral("sw_pcnumber_lineedit"));
        sw_pcnumber_lineedit->setGeometry(QRect(170, 450, 31, 20));
        sw_horizontalacq_radiobutton = new QRadioButton(centralwidget);
        sw_horizontalacq_radiobutton->setObjectName(QStringLiteral("sw_horizontalacq_radiobutton"));
        sw_horizontalacq_radiobutton->setGeometry(QRect(30, 30, 82, 17));
        sw_verticalacq_radiobutton = new QRadioButton(centralwidget);
        sw_verticalacq_radiobutton->setObjectName(QStringLiteral("sw_verticalacq_radiobutton"));
        sw_verticalacq_radiobutton->setGeometry(QRect(120, 30, 82, 17));
        sw_acqtype_label = new QLabel(centralwidget);
        sw_acqtype_label->setObjectName(QStringLiteral("sw_acqtype_label"));
        sw_acqtype_label->setGeometry(QRect(20, 10, 131, 16));
        sw_qvtkWidget = new QVTKWidget(centralwidget);
        sw_qvtkWidget->setObjectName(QStringLiteral("sw_qvtkWidget"));
        sw_qvtkWidget->setGeometry(QRect(240, 50, 421, 341));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(sw_qvtkWidget->sizePolicy().hasHeightForWidth());
        sw_qvtkWidget->setSizePolicy(sizePolicy);
        scanwindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(scanwindow);
        menubar->setObjectName(QStringLiteral("menubar"));
        menubar->setGeometry(QRect(0, 0, 700, 21));
        scanwindow->setMenuBar(menubar);
        statusbar = new QStatusBar(scanwindow);
        statusbar->setObjectName(QStringLiteral("statusbar"));
        scanwindow->setStatusBar(statusbar);

        retranslateUi(scanwindow);

        QMetaObject::connectSlotsByName(scanwindow);
    } // setupUi

    void retranslateUi(QMainWindow *scanwindow)
    {
        scanwindow->setWindowTitle(QApplication::translate("scanwindow", "MainWindow", 0));
        sw_scanparameters_label->setText(QApplication::translate("scanwindow", "Scan parameters", 0));
        sw_xmin_label->setText(QApplication::translate("scanwindow", "X minimum", 0));
        sw_xmax_label->setText(QApplication::translate("scanwindow", "X maximum", 0));
        sw_ymin_label->setText(QApplication::translate("scanwindow", "Y minimum", 0));
        sw_ymax_label->setText(QApplication::translate("scanwindow", "Y maximum", 0));
        sw_zmin_label->setText(QApplication::translate("scanwindow", "Z minimum", 0));
        sw_zmax_label->setText(QApplication::translate("scanwindow", "Z maximum", 0));
        sw_filtering_checkbox->setText(QApplication::translate("scanwindow", "Filter coordinates", 0));
        sw_startscan_pushbutton->setText(QApplication::translate("scanwindow", "Start scan", 0));
        sw_stopscan_pushbutton->setText(QApplication::translate("scanwindow", "Stop scan", 0));
        sw_visualizer_label->setText(QApplication::translate("scanwindow", "Visualizer", 0));
        sw_capturepc_puchbutton->setText(QApplication::translate("scanwindow", "Capture point cloud", 0));
        sw_pcnumber_label->setText(QApplication::translate("scanwindow", "Number of captured PCs", 0));
        sw_pcnumber_lineedit->setText(QApplication::translate("scanwindow", "0", 0));
        sw_horizontalacq_radiobutton->setText(QApplication::translate("scanwindow", "Horizontal", 0));
        sw_verticalacq_radiobutton->setText(QApplication::translate("scanwindow", "Vertical", 0));
        sw_acqtype_label->setText(QApplication::translate("scanwindow", "Acquisition mode", 0));
    } // retranslateUi

};

namespace Ui {
    class scanwindow: public Ui_scanwindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SCANWINDOW_H
