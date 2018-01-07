/********************************************************************************
** Form generated from reading UI file 'filterwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.7.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_FILTERWINDOW_H
#define UI_FILTERWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpinBox>

QT_BEGIN_NAMESPACE

class Ui_filterwindow
{
public:
    QPushButton *filter_pb;
    QLabel *label;
    QLabel *label_2;
    QDoubleSpinBox *radius_sb;
    QLabel *label_4;
    QFrame *line;
    QCheckBox *voxelgrid_cb;
    QLabel *label_5;
    QLabel *label_6;
    QLabel *label_7;
    QDoubleSpinBox *x_sb;
    QDoubleSpinBox *y_sb;
    QDoubleSpinBox *z_sb;
    QLabel *label_8;
    QLabel *label_9;
    QLabel *label_10;
    QFrame *line_2;
    QCheckBox *bilateral_cb;
    QLabel *label_11;
    QLabel *label_12;
    QSpinBox *sigmas_sb;
    QDoubleSpinBox *sigmar_sb;
    QLabel *label_13;
    QFrame *line_3;
    QFrame *line_4;
    QCheckBox *median_cb;
    QSpinBox *windowsize_sb;
    QDoubleSpinBox *maxmov_sb;
    QLabel *label_14;
    QLabel *label_15;
    QLabel *label_16;
    QCheckBox *random_cb;
    QSpinBox *randord_sb;
    QLabel *label_17;
    QFrame *line_5;
    QCheckBox *normal_cb;
    QLabel *label_18;
    QSpinBox *normord_sb;
    QLabel *label_19;
    QSpinBox *normbins_sb;
    QFrame *line_6;
    QCheckBox *covar_cb;
    QLabel *label_20;
    QSpinBox *covarord_sb;
    QLabel *label_22;
    QLabel *label_23;
    QLabel *label_24;
    QLabel *label_25;
    QLabel *label_26;
    QLabel *label_27;

    void setupUi(QDialog *filterwindow)
    {
        if (filterwindow->objectName().isEmpty())
            filterwindow->setObjectName(QStringLiteral("filterwindow"));
        filterwindow->resize(730, 450);
        filter_pb = new QPushButton(filterwindow);
        filter_pb->setObjectName(QStringLiteral("filter_pb"));
        filter_pb->setGeometry(QRect(210, 400, 301, 31));
        label = new QLabel(filterwindow);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(10, 10, 171, 16));
        label_2 = new QLabel(filterwindow);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(70, 60, 121, 16));
        radius_sb = new QDoubleSpinBox(filterwindow);
        radius_sb->setObjectName(QStringLiteral("radius_sb"));
        radius_sb->setGeometry(QRect(200, 60, 62, 22));
        radius_sb->setMinimum(0.01);
        radius_sb->setMaximum(0.1);
        radius_sb->setSingleStep(0.01);
        radius_sb->setValue(0.05);
        label_4 = new QLabel(filterwindow);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setGeometry(QRect(270, 60, 81, 16));
        line = new QFrame(filterwindow);
        line->setObjectName(QStringLiteral("line"));
        line->setGeometry(QRect(-10, 100, 781, 20));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);
        voxelgrid_cb = new QCheckBox(filterwindow);
        voxelgrid_cb->setObjectName(QStringLiteral("voxelgrid_cb"));
        voxelgrid_cb->setGeometry(QRect(10, 120, 91, 17));
        label_5 = new QLabel(filterwindow);
        label_5->setObjectName(QStringLiteral("label_5"));
        label_5->setGeometry(QRect(150, 150, 47, 13));
        label_6 = new QLabel(filterwindow);
        label_6->setObjectName(QStringLiteral("label_6"));
        label_6->setGeometry(QRect(150, 180, 47, 13));
        label_7 = new QLabel(filterwindow);
        label_7->setObjectName(QStringLiteral("label_7"));
        label_7->setGeometry(QRect(150, 210, 47, 13));
        x_sb = new QDoubleSpinBox(filterwindow);
        x_sb->setObjectName(QStringLiteral("x_sb"));
        x_sb->setGeometry(QRect(200, 150, 62, 22));
        x_sb->setMinimum(1);
        x_sb->setMaximum(5);
        x_sb->setSingleStep(0.5);
        x_sb->setValue(3);
        y_sb = new QDoubleSpinBox(filterwindow);
        y_sb->setObjectName(QStringLiteral("y_sb"));
        y_sb->setGeometry(QRect(200, 180, 62, 22));
        y_sb->setMinimum(1);
        y_sb->setMaximum(5);
        y_sb->setSingleStep(0.5);
        y_sb->setValue(3);
        z_sb = new QDoubleSpinBox(filterwindow);
        z_sb->setObjectName(QStringLiteral("z_sb"));
        z_sb->setGeometry(QRect(200, 210, 62, 22));
        z_sb->setMinimum(1);
        z_sb->setMaximum(5);
        z_sb->setSingleStep(0.5);
        z_sb->setValue(3);
        label_8 = new QLabel(filterwindow);
        label_8->setObjectName(QStringLiteral("label_8"));
        label_8->setGeometry(QRect(270, 150, 81, 16));
        label_9 = new QLabel(filterwindow);
        label_9->setObjectName(QStringLiteral("label_9"));
        label_9->setGeometry(QRect(270, 180, 81, 16));
        label_10 = new QLabel(filterwindow);
        label_10->setObjectName(QStringLiteral("label_10"));
        label_10->setGeometry(QRect(270, 210, 81, 16));
        line_2 = new QFrame(filterwindow);
        line_2->setObjectName(QStringLiteral("line_2"));
        line_2->setGeometry(QRect(-50, 240, 411, 20));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);
        bilateral_cb = new QCheckBox(filterwindow);
        bilateral_cb->setObjectName(QStringLiteral("bilateral_cb"));
        bilateral_cb->setEnabled(true);
        bilateral_cb->setGeometry(QRect(10, 260, 111, 17));
        label_11 = new QLabel(filterwindow);
        label_11->setObjectName(QStringLiteral("label_11"));
        label_11->setGeometry(QRect(70, 290, 47, 13));
        label_12 = new QLabel(filterwindow);
        label_12->setObjectName(QStringLiteral("label_12"));
        label_12->setGeometry(QRect(70, 320, 47, 13));
        sigmas_sb = new QSpinBox(filterwindow);
        sigmas_sb->setObjectName(QStringLiteral("sigmas_sb"));
        sigmas_sb->setGeometry(QRect(200, 290, 61, 22));
        sigmas_sb->setMinimum(1);
        sigmas_sb->setMaximum(20);
        sigmas_sb->setValue(5);
        sigmar_sb = new QDoubleSpinBox(filterwindow);
        sigmar_sb->setObjectName(QStringLiteral("sigmar_sb"));
        sigmar_sb->setGeometry(QRect(200, 320, 62, 22));
        sigmar_sb->setMinimum(0.01);
        sigmar_sb->setMaximum(5);
        sigmar_sb->setSingleStep(0.01);
        sigmar_sb->setValue(0.05);
        label_13 = new QLabel(filterwindow);
        label_13->setObjectName(QStringLiteral("label_13"));
        label_13->setGeometry(QRect(270, 320, 81, 16));
        line_3 = new QFrame(filterwindow);
        line_3->setObjectName(QStringLiteral("line_3"));
        line_3->setGeometry(QRect(350, 0, 20, 381));
        line_3->setFrameShape(QFrame::VLine);
        line_3->setFrameShadow(QFrame::Sunken);
        line_4 = new QFrame(filterwindow);
        line_4->setObjectName(QStringLiteral("line_4"));
        line_4->setGeometry(QRect(-30, 370, 791, 20));
        line_4->setFrameShape(QFrame::HLine);
        line_4->setFrameShadow(QFrame::Sunken);
        median_cb = new QCheckBox(filterwindow);
        median_cb->setObjectName(QStringLiteral("median_cb"));
        median_cb->setEnabled(true);
        median_cb->setGeometry(QRect(370, 10, 121, 17));
        windowsize_sb = new QSpinBox(filterwindow);
        windowsize_sb->setObjectName(QStringLiteral("windowsize_sb"));
        windowsize_sb->setGeometry(QRect(560, 40, 61, 22));
        windowsize_sb->setMinimum(1);
        windowsize_sb->setMaximum(20);
        windowsize_sb->setValue(5);
        maxmov_sb = new QDoubleSpinBox(filterwindow);
        maxmov_sb->setObjectName(QStringLiteral("maxmov_sb"));
        maxmov_sb->setGeometry(QRect(560, 70, 62, 22));
        maxmov_sb->setMinimum(1);
        maxmov_sb->setMaximum(10);
        label_14 = new QLabel(filterwindow);
        label_14->setObjectName(QStringLiteral("label_14"));
        label_14->setGeometry(QRect(450, 40, 91, 16));
        label_15 = new QLabel(filterwindow);
        label_15->setObjectName(QStringLiteral("label_15"));
        label_15->setGeometry(QRect(640, 70, 81, 16));
        label_16 = new QLabel(filterwindow);
        label_16->setObjectName(QStringLiteral("label_16"));
        label_16->setGeometry(QRect(450, 70, 101, 16));
        random_cb = new QCheckBox(filterwindow);
        random_cb->setObjectName(QStringLiteral("random_cb"));
        random_cb->setGeometry(QRect(370, 120, 111, 17));
        randord_sb = new QSpinBox(filterwindow);
        randord_sb->setObjectName(QStringLiteral("randord_sb"));
        randord_sb->setGeometry(QRect(560, 150, 61, 22));
        randord_sb->setMinimum(2);
        randord_sb->setMaximum(10);
        randord_sb->setValue(5);
        label_17 = new QLabel(filterwindow);
        label_17->setObjectName(QStringLiteral("label_17"));
        label_17->setGeometry(QRect(450, 150, 91, 16));
        line_5 = new QFrame(filterwindow);
        line_5->setObjectName(QStringLiteral("line_5"));
        line_5->setGeometry(QRect(360, 180, 411, 20));
        line_5->setFrameShape(QFrame::HLine);
        line_5->setFrameShadow(QFrame::Sunken);
        normal_cb = new QCheckBox(filterwindow);
        normal_cb->setObjectName(QStringLiteral("normal_cb"));
        normal_cb->setEnabled(true);
        normal_cb->setGeometry(QRect(370, 200, 111, 17));
        label_18 = new QLabel(filterwindow);
        label_18->setObjectName(QStringLiteral("label_18"));
        label_18->setGeometry(QRect(450, 230, 91, 16));
        normord_sb = new QSpinBox(filterwindow);
        normord_sb->setObjectName(QStringLiteral("normord_sb"));
        normord_sb->setGeometry(QRect(560, 230, 61, 22));
        normord_sb->setMinimum(2);
        normord_sb->setMaximum(10);
        normord_sb->setValue(5);
        label_19 = new QLabel(filterwindow);
        label_19->setObjectName(QStringLiteral("label_19"));
        label_19->setGeometry(QRect(450, 260, 81, 16));
        normbins_sb = new QSpinBox(filterwindow);
        normbins_sb->setObjectName(QStringLiteral("normbins_sb"));
        normbins_sb->setGeometry(QRect(560, 260, 61, 22));
        normbins_sb->setMinimum(8);
        normbins_sb->setMaximum(32);
        line_6 = new QFrame(filterwindow);
        line_6->setObjectName(QStringLiteral("line_6"));
        line_6->setGeometry(QRect(360, 290, 401, 20));
        line_6->setFrameShape(QFrame::HLine);
        line_6->setFrameShadow(QFrame::Sunken);
        covar_cb = new QCheckBox(filterwindow);
        covar_cb->setObjectName(QStringLiteral("covar_cb"));
        covar_cb->setEnabled(false);
        covar_cb->setGeometry(QRect(370, 310, 151, 17));
        label_20 = new QLabel(filterwindow);
        label_20->setObjectName(QStringLiteral("label_20"));
        label_20->setGeometry(QRect(450, 340, 91, 16));
        covarord_sb = new QSpinBox(filterwindow);
        covarord_sb->setObjectName(QStringLiteral("covarord_sb"));
        covarord_sb->setGeometry(QRect(560, 340, 61, 22));
        covarord_sb->setMinimum(2);
        covarord_sb->setMaximum(10);
        covarord_sb->setValue(5);
        label_22 = new QLabel(filterwindow);
        label_22->setObjectName(QStringLiteral("label_22"));
        label_22->setGeometry(QRect(270, 290, 81, 16));
        label_23 = new QLabel(filterwindow);
        label_23->setObjectName(QStringLiteral("label_23"));
        label_23->setGeometry(QRect(640, 40, 81, 16));
        label_24 = new QLabel(filterwindow);
        label_24->setObjectName(QStringLiteral("label_24"));
        label_24->setGeometry(QRect(640, 150, 81, 16));
        label_25 = new QLabel(filterwindow);
        label_25->setObjectName(QStringLiteral("label_25"));
        label_25->setGeometry(QRect(640, 230, 81, 16));
        label_26 = new QLabel(filterwindow);
        label_26->setObjectName(QStringLiteral("label_26"));
        label_26->setGeometry(QRect(640, 340, 81, 16));
        label_27 = new QLabel(filterwindow);
        label_27->setObjectName(QStringLiteral("label_27"));
        label_27->setGeometry(QRect(640, 260, 81, 16));

        retranslateUi(filterwindow);

        QMetaObject::connectSlotsByName(filterwindow);
    } // setupUi

    void retranslateUi(QDialog *filterwindow)
    {
        filterwindow->setWindowTitle(QApplication::translate("filterwindow", "Dialog", 0));
        filter_pb->setText(QApplication::translate("filterwindow", "Filter", 0));
        label->setText(QApplication::translate("filterwindow", "Normal computation parameters", 0));
        label_2->setText(QApplication::translate("filterwindow", "Radius", 0));
        label_4->setText(QApplication::translate("filterwindow", "m [0.01-0.1]", 0));
        voxelgrid_cb->setText(QApplication::translate("filterwindow", "Voxel Grid", 0));
        label_5->setText(QApplication::translate("filterwindow", "x", 0));
        label_6->setText(QApplication::translate("filterwindow", "y", 0));
        label_7->setText(QApplication::translate("filterwindow", "z", 0));
        label_8->setText(QApplication::translate("filterwindow", "cm [1-5]", 0));
        label_9->setText(QApplication::translate("filterwindow", "cm [1-5]", 0));
        label_10->setText(QApplication::translate("filterwindow", "cm [1-5]", 0));
        bilateral_cb->setText(QApplication::translate("filterwindow", "Bilateral filter", 0));
        label_11->setText(QApplication::translate("filterwindow", "Sigma S", 0));
        label_12->setText(QApplication::translate("filterwindow", "Sigma R", 0));
        label_13->setText(QApplication::translate("filterwindow", "cm [0.01-1]", 0));
        median_cb->setText(QApplication::translate("filterwindow", "Median filter", 0));
        label_14->setText(QApplication::translate("filterwindow", "Window size", 0));
        label_15->setText(QApplication::translate("filterwindow", "cm [1-10]", 0));
        label_16->setText(QApplication::translate("filterwindow", "Maximum movement", 0));
        random_cb->setText(QApplication::translate("filterwindow", "Random sampling", 0));
        label_17->setText(QApplication::translate("filterwindow", "Sampling ratio", 0));
        normal_cb->setText(QApplication::translate("filterwindow", "Normal sampling", 0));
        label_18->setText(QApplication::translate("filterwindow", "Sampling ratio", 0));
        label_19->setText(QApplication::translate("filterwindow", "Number of bins", 0));
        covar_cb->setText(QApplication::translate("filterwindow", "Covariance sampling", 0));
        label_20->setText(QApplication::translate("filterwindow", "Sampling ratio", 0));
        label_22->setText(QApplication::translate("filterwindow", "[1-20]", 0));
        label_23->setText(QApplication::translate("filterwindow", "[1-20]", 0));
        label_24->setText(QApplication::translate("filterwindow", "[2-10]", 0));
        label_25->setText(QApplication::translate("filterwindow", "[2-10]", 0));
        label_26->setText(QApplication::translate("filterwindow", "[2-10]", 0));
        label_27->setText(QApplication::translate("filterwindow", "[8-32]", 0));
    } // retranslateUi

};

namespace Ui {
    class filterwindow: public Ui_filterwindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_FILTERWINDOW_H
