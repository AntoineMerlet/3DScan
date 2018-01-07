/********************************************************************************
** Form generated from reading UI file 'regwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.7.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_REGWINDOW_H
#define UI_REGWINDOW_H

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
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QSpinBox>

QT_BEGIN_NAMESPACE

class Ui_regwindow
{
public:
    QPushButton *reg_button;
    QLabel *label;
    QLabel *label_2;
    QDoubleSpinBox *radius_sb;
    QFrame *line;
    QFrame *line_2;
    QLabel *label_4;
    QLabel *label_5;
    QDoubleSpinBox *dist_sb;
    QFrame *line_3;
    QLabel *label_6;
    QCheckBox *median_cb;
    QLabel *label_7;
    QDoubleSpinBox *medianfact_sb;
    QCheckBox *one2one_cb;
    QCheckBox *ransac_cb;
    QLabel *label_8;
    QLabel *label_9;
    QSpinBox *ransacit_sb;
    QDoubleSpinBox *ransacth_sb;
    QCheckBox *surface_cb;
    QLabel *label_10;
    QSpinBox *angle_sb;
    QCheckBox *boundary_cb;
    QFrame *line_4;
    QLabel *label_11;
    QRadioButton *p2plls;
    QRadioButton *svd;
    QRadioButton *lmp2p;
    QRadioButton *lmp2s;
    QPushButton *default_button;
    QFrame *line_5;
    QLabel *label_12;
    QLabel *label_13;
    QCheckBox *similar_cb;
    QCheckBox *relativemse_cb;
    QCheckBox *absolutemse_cb;
    QSpinBox *maxiter_sb;
    QSpinBox *similariter_sb;
    QDoubleSpinBox *relativemse_sb;
    QDoubleSpinBox *absolutemse_sb;
    QLabel *label_14;
    QLabel *label_16;
    QLabel *label_17;
    QLabel *label_18;
    QLabel *label_19;
    QLabel *label_20;
    QLabel *label_21;
    QLabel *label_22;

    void setupUi(QDialog *regwindow)
    {
        if (regwindow->objectName().isEmpty())
            regwindow->setObjectName(QStringLiteral("regwindow"));
        regwindow->resize(700, 470);
        reg_button = new QPushButton(regwindow);
        reg_button->setObjectName(QStringLiteral("reg_button"));
        reg_button->setGeometry(QRect(20, 430, 321, 23));
        label = new QLabel(regwindow);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(10, 10, 171, 16));
        label_2 = new QLabel(regwindow);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(70, 70, 131, 16));
        radius_sb = new QDoubleSpinBox(regwindow);
        radius_sb->setObjectName(QStringLiteral("radius_sb"));
        radius_sb->setGeometry(QRect(200, 70, 62, 22));
        radius_sb->setMinimum(0.01);
        radius_sb->setMaximum(0.1);
        radius_sb->setSingleStep(0.01);
        radius_sb->setValue(0.05);
        line = new QFrame(regwindow);
        line->setObjectName(QStringLiteral("line"));
        line->setGeometry(QRect(350, -10, 20, 331));
        line->setFrameShape(QFrame::VLine);
        line->setFrameShadow(QFrame::Sunken);
        line_2 = new QFrame(regwindow);
        line_2->setObjectName(QStringLiteral("line_2"));
        line_2->setGeometry(QRect(0, 110, 361, 16));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);
        label_4 = new QLabel(regwindow);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setGeometry(QRect(10, 130, 151, 16));
        label_5 = new QLabel(regwindow);
        label_5->setObjectName(QStringLiteral("label_5"));
        label_5->setGeometry(QRect(70, 160, 121, 16));
        dist_sb = new QDoubleSpinBox(regwindow);
        dist_sb->setObjectName(QStringLiteral("dist_sb"));
        dist_sb->setGeometry(QRect(200, 160, 62, 22));
        dist_sb->setMinimum(1);
        dist_sb->setMaximum(100);
        dist_sb->setSingleStep(0.5);
        dist_sb->setValue(5);
        line_3 = new QFrame(regwindow);
        line_3->setObjectName(QStringLiteral("line_3"));
        line_3->setGeometry(QRect(0, 190, 361, 16));
        line_3->setFrameShape(QFrame::HLine);
        line_3->setFrameShadow(QFrame::Sunken);
        label_6 = new QLabel(regwindow);
        label_6->setObjectName(QStringLiteral("label_6"));
        label_6->setGeometry(QRect(370, 10, 141, 16));
        median_cb = new QCheckBox(regwindow);
        median_cb->setObjectName(QStringLiteral("median_cb"));
        median_cb->setGeometry(QRect(390, 40, 121, 17));
        label_7 = new QLabel(regwindow);
        label_7->setObjectName(QStringLiteral("label_7"));
        label_7->setGeometry(QRect(440, 70, 91, 16));
        medianfact_sb = new QDoubleSpinBox(regwindow);
        medianfact_sb->setObjectName(QStringLiteral("medianfact_sb"));
        medianfact_sb->setGeometry(QRect(540, 70, 62, 22));
        medianfact_sb->setMinimum(1);
        medianfact_sb->setMaximum(10);
        medianfact_sb->setSingleStep(0.25);
        medianfact_sb->setValue(2);
        one2one_cb = new QCheckBox(regwindow);
        one2one_cb->setObjectName(QStringLiteral("one2one_cb"));
        one2one_cb->setEnabled(false);
        one2one_cb->setGeometry(QRect(390, 190, 70, 17));
        ransac_cb = new QCheckBox(regwindow);
        ransac_cb->setObjectName(QStringLiteral("ransac_cb"));
        ransac_cb->setEnabled(false);
        ransac_cb->setGeometry(QRect(390, 230, 70, 17));
        label_8 = new QLabel(regwindow);
        label_8->setObjectName(QStringLiteral("label_8"));
        label_8->setGeometry(QRect(440, 260, 91, 16));
        label_9 = new QLabel(regwindow);
        label_9->setObjectName(QStringLiteral("label_9"));
        label_9->setGeometry(QRect(440, 290, 91, 16));
        ransacit_sb = new QSpinBox(regwindow);
        ransacit_sb->setObjectName(QStringLiteral("ransacit_sb"));
        ransacit_sb->setEnabled(false);
        ransacit_sb->setGeometry(QRect(540, 260, 61, 22));
        ransacit_sb->setMinimum(10);
        ransacit_sb->setMaximum(1000);
        ransacit_sb->setValue(200);
        ransacth_sb = new QDoubleSpinBox(regwindow);
        ransacth_sb->setObjectName(QStringLiteral("ransacth_sb"));
        ransacth_sb->setEnabled(false);
        ransacth_sb->setGeometry(QRect(540, 290, 62, 22));
        ransacth_sb->setMinimum(1);
        ransacth_sb->setMaximum(10);
        ransacth_sb->setSingleStep(0.5);
        ransacth_sb->setValue(5);
        surface_cb = new QCheckBox(regwindow);
        surface_cb->setObjectName(QStringLiteral("surface_cb"));
        surface_cb->setEnabled(false);
        surface_cb->setGeometry(QRect(390, 90, 111, 17));
        label_10 = new QLabel(regwindow);
        label_10->setObjectName(QStringLiteral("label_10"));
        label_10->setGeometry(QRect(440, 120, 71, 16));
        angle_sb = new QSpinBox(regwindow);
        angle_sb->setObjectName(QStringLiteral("angle_sb"));
        angle_sb->setEnabled(false);
        angle_sb->setGeometry(QRect(540, 120, 61, 22));
        angle_sb->setMaximum(360);
        angle_sb->setValue(45);
        boundary_cb = new QCheckBox(regwindow);
        boundary_cb->setObjectName(QStringLiteral("boundary_cb"));
        boundary_cb->setEnabled(false);
        boundary_cb->setGeometry(QRect(390, 150, 131, 17));
        line_4 = new QFrame(regwindow);
        line_4->setObjectName(QStringLiteral("line_4"));
        line_4->setGeometry(QRect(-30, 410, 731, 20));
        line_4->setFrameShape(QFrame::HLine);
        line_4->setFrameShadow(QFrame::Sunken);
        label_11 = new QLabel(regwindow);
        label_11->setObjectName(QStringLiteral("label_11"));
        label_11->setGeometry(QRect(10, 210, 121, 16));
        p2plls = new QRadioButton(regwindow);
        p2plls->setObjectName(QStringLiteral("p2plls"));
        p2plls->setGeometry(QRect(30, 240, 111, 17));
        p2plls->setChecked(true);
        svd = new QRadioButton(regwindow);
        svd->setObjectName(QStringLiteral("svd"));
        svd->setGeometry(QRect(170, 240, 82, 17));
        lmp2p = new QRadioButton(regwindow);
        lmp2p->setObjectName(QStringLiteral("lmp2p"));
        lmp2p->setGeometry(QRect(30, 280, 101, 17));
        lmp2s = new QRadioButton(regwindow);
        lmp2s->setObjectName(QStringLiteral("lmp2s"));
        lmp2s->setGeometry(QRect(170, 280, 111, 17));
        default_button = new QPushButton(regwindow);
        default_button->setObjectName(QStringLiteral("default_button"));
        default_button->setGeometry(QRect(374, 430, 301, 23));
        line_5 = new QFrame(regwindow);
        line_5->setObjectName(QStringLiteral("line_5"));
        line_5->setGeometry(QRect(-20, 310, 731, 20));
        line_5->setFrameShape(QFrame::HLine);
        line_5->setFrameShadow(QFrame::Sunken);
        label_12 = new QLabel(regwindow);
        label_12->setObjectName(QStringLiteral("label_12"));
        label_12->setGeometry(QRect(10, 330, 131, 16));
        label_12->setFrameShape(QFrame::NoFrame);
        label_13 = new QLabel(regwindow);
        label_13->setObjectName(QStringLiteral("label_13"));
        label_13->setGeometry(QRect(40, 360, 161, 16));
        similar_cb = new QCheckBox(regwindow);
        similar_cb->setObjectName(QStringLiteral("similar_cb"));
        similar_cb->setGeometry(QRect(20, 390, 201, 17));
        relativemse_cb = new QCheckBox(regwindow);
        relativemse_cb->setObjectName(QStringLiteral("relativemse_cb"));
        relativemse_cb->setGeometry(QRect(380, 360, 111, 17));
        absolutemse_cb = new QCheckBox(regwindow);
        absolutemse_cb->setObjectName(QStringLiteral("absolutemse_cb"));
        absolutemse_cb->setGeometry(QRect(380, 390, 101, 17));
        maxiter_sb = new QSpinBox(regwindow);
        maxiter_sb->setObjectName(QStringLiteral("maxiter_sb"));
        maxiter_sb->setGeometry(QRect(230, 360, 61, 22));
        maxiter_sb->setMinimum(10);
        maxiter_sb->setMaximum(1000);
        maxiter_sb->setValue(200);
        similariter_sb = new QSpinBox(regwindow);
        similariter_sb->setObjectName(QStringLiteral("similariter_sb"));
        similariter_sb->setGeometry(QRect(230, 390, 61, 22));
        similariter_sb->setMinimum(3);
        similariter_sb->setMaximum(20);
        relativemse_sb = new QDoubleSpinBox(regwindow);
        relativemse_sb->setObjectName(QStringLiteral("relativemse_sb"));
        relativemse_sb->setGeometry(QRect(540, 360, 62, 22));
        absolutemse_sb = new QDoubleSpinBox(regwindow);
        absolutemse_sb->setObjectName(QStringLiteral("absolutemse_sb"));
        absolutemse_sb->setGeometry(QRect(540, 390, 62, 22));
        label_14 = new QLabel(regwindow);
        label_14->setObjectName(QStringLiteral("label_14"));
        label_14->setGeometry(QRect(270, 70, 61, 16));
        label_16 = new QLabel(regwindow);
        label_16->setObjectName(QStringLiteral("label_16"));
        label_16->setGeometry(QRect(270, 160, 61, 16));
        label_17 = new QLabel(regwindow);
        label_17->setObjectName(QStringLiteral("label_17"));
        label_17->setGeometry(QRect(610, 70, 61, 16));
        label_18 = new QLabel(regwindow);
        label_18->setObjectName(QStringLiteral("label_18"));
        label_18->setGeometry(QRect(610, 260, 61, 16));
        label_19 = new QLabel(regwindow);
        label_19->setObjectName(QStringLiteral("label_19"));
        label_19->setGeometry(QRect(610, 290, 61, 16));
        label_20 = new QLabel(regwindow);
        label_20->setObjectName(QStringLiteral("label_20"));
        label_20->setGeometry(QRect(610, 120, 61, 16));
        label_21 = new QLabel(regwindow);
        label_21->setObjectName(QStringLiteral("label_21"));
        label_21->setGeometry(QRect(300, 360, 61, 16));
        label_22 = new QLabel(regwindow);
        label_22->setObjectName(QStringLiteral("label_22"));
        label_22->setGeometry(QRect(300, 390, 61, 16));

        retranslateUi(regwindow);

        QMetaObject::connectSlotsByName(regwindow);
    } // setupUi

    void retranslateUi(QDialog *regwindow)
    {
        regwindow->setWindowTitle(QApplication::translate("regwindow", "Dialog", 0));
        reg_button->setText(QApplication::translate("regwindow", "Register", 0));
        label->setText(QApplication::translate("regwindow", "Normal computation parameters", 0));
        label_2->setText(QApplication::translate("regwindow", "Radius", 0));
        label_4->setText(QApplication::translate("regwindow", "Correspondance estimation", 0));
        label_5->setText(QApplication::translate("regwindow", "Maximum distance", 0));
        label_6->setText(QApplication::translate("regwindow", "Rejection", 0));
        median_cb->setText(QApplication::translate("regwindow", "Median distance", 0));
        label_7->setText(QApplication::translate("regwindow", "Median factor", 0));
        one2one_cb->setText(QApplication::translate("regwindow", "1 to 1", 0));
        ransac_cb->setText(QApplication::translate("regwindow", "Ransac", 0));
        label_8->setText(QApplication::translate("regwindow", "Iterations", 0));
        label_9->setText(QApplication::translate("regwindow", "Inlier threshold", 0));
        surface_cb->setText(QApplication::translate("regwindow", "Surface normal", 0));
        label_10->setText(QApplication::translate("regwindow", "Angle", 0));
        boundary_cb->setText(QApplication::translate("regwindow", "Organized boundary", 0));
        label_11->setText(QApplication::translate("regwindow", "Select a method", 0));
        p2plls->setText(QApplication::translate("regwindow", "Point2Plane LLS", 0));
        svd->setText(QApplication::translate("regwindow", "SVD", 0));
        lmp2p->setText(QApplication::translate("regwindow", "LM Point2Point", 0));
        lmp2s->setText(QApplication::translate("regwindow", "LM Point2Surface", 0));
        default_button->setText(QApplication::translate("regwindow", "Default", 0));
        label_12->setText(QApplication::translate("regwindow", "Method parameters", 0));
        label_13->setText(QApplication::translate("regwindow", "Maximum number of iterations", 0));
        similar_cb->setText(QApplication::translate("regwindow", "Maximum number of similar iterations", 0));
        relativemse_cb->setText(QApplication::translate("regwindow", "Relative MSE", 0));
        absolutemse_cb->setText(QApplication::translate("regwindow", "Absolute MSE", 0));
        label_14->setText(QApplication::translate("regwindow", "m [0.01-0.1]", 0));
        label_16->setText(QApplication::translate("regwindow", "cm [1-100]", 0));
        label_17->setText(QApplication::translate("regwindow", "[1-10]", 0));
        label_18->setText(QApplication::translate("regwindow", "[10-1000]", 0));
        label_19->setText(QApplication::translate("regwindow", "cm [1-10]", 0));
        label_20->setText(QApplication::translate("regwindow", "deg [0-360]", 0));
        label_21->setText(QApplication::translate("regwindow", "[10-1000]", 0));
        label_22->setText(QApplication::translate("regwindow", "[3-20]", 0));
    } // retranslateUi

};

namespace Ui {
    class regwindow: public Ui_regwindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_REGWINDOW_H
