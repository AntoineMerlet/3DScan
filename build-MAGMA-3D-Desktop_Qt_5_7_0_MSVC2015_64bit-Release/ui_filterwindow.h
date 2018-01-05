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
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QPushButton>

QT_BEGIN_NAMESPACE

class Ui_filterwindow
{
public:
    QCheckBox *bi_filt;
    QCheckBox *vox_filt;
    QPushButton *filt_button;

    void setupUi(QDialog *filterwindow)
    {
        if (filterwindow->objectName().isEmpty())
            filterwindow->setObjectName(QStringLiteral("filterwindow"));
        filterwindow->resize(450, 300);
        bi_filt = new QCheckBox(filterwindow);
        bi_filt->setObjectName(QStringLiteral("bi_filt"));
        bi_filt->setGeometry(QRect(20, 20, 101, 17));
        vox_filt = new QCheckBox(filterwindow);
        vox_filt->setObjectName(QStringLiteral("vox_filt"));
        vox_filt->setGeometry(QRect(20, 90, 101, 17));
        filt_button = new QPushButton(filterwindow);
        filt_button->setObjectName(QStringLiteral("filt_button"));
        filt_button->setGeometry(QRect(190, 260, 75, 23));

        retranslateUi(filterwindow);

        QMetaObject::connectSlotsByName(filterwindow);
    } // setupUi

    void retranslateUi(QDialog *filterwindow)
    {
        filterwindow->setWindowTitle(QApplication::translate("filterwindow", "Dialog", 0));
        bi_filt->setText(QApplication::translate("filterwindow", "Bilateral filter", 0));
        vox_filt->setText(QApplication::translate("filterwindow", "Voxelgrid filter", 0));
        filt_button->setText(QApplication::translate("filterwindow", "Filter", 0));
    } // retranslateUi

};

namespace Ui {
    class filterwindow: public Ui_filterwindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_FILTERWINDOW_H
