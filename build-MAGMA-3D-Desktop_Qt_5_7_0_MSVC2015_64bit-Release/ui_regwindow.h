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
#include <QtWidgets/QDialog>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QPushButton>

QT_BEGIN_NAMESPACE

class Ui_regwindow
{
public:
    QPushButton *reg_button;

    void setupUi(QDialog *regwindow)
    {
        if (regwindow->objectName().isEmpty())
            regwindow->setObjectName(QStringLiteral("regwindow"));
        regwindow->resize(665, 450);
        reg_button = new QPushButton(regwindow);
        reg_button->setObjectName(QStringLiteral("reg_button"));
        reg_button->setGeometry(QRect(290, 410, 75, 23));

        retranslateUi(regwindow);

        QMetaObject::connectSlotsByName(regwindow);
    } // setupUi

    void retranslateUi(QDialog *regwindow)
    {
        regwindow->setWindowTitle(QApplication::translate("regwindow", "Dialog", 0));
        reg_button->setText(QApplication::translate("regwindow", "Register", 0));
    } // retranslateUi

};

namespace Ui {
    class regwindow: public Ui_regwindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_REGWINDOW_H
