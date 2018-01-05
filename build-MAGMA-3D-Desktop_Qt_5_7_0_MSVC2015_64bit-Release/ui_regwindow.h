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
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QPushButton>

QT_BEGIN_NAMESPACE

class Ui_regwindow
{
public:
    QCheckBox *icp_reg;
    QPushButton *reg_button;

    void setupUi(QDialog *regwindow)
    {
        if (regwindow->objectName().isEmpty())
            regwindow->setObjectName(QStringLiteral("regwindow"));
        regwindow->resize(450, 300);
        icp_reg = new QCheckBox(regwindow);
        icp_reg->setObjectName(QStringLiteral("icp_reg"));
        icp_reg->setGeometry(QRect(20, 20, 121, 17));
        reg_button = new QPushButton(regwindow);
        reg_button->setObjectName(QStringLiteral("reg_button"));
        reg_button->setGeometry(QRect(190, 260, 75, 23));

        retranslateUi(regwindow);

        QMetaObject::connectSlotsByName(regwindow);
    } // setupUi

    void retranslateUi(QDialog *regwindow)
    {
        regwindow->setWindowTitle(QApplication::translate("regwindow", "Dialog", 0));
        icp_reg->setText(QApplication::translate("regwindow", "ICP", 0));
        reg_button->setText(QApplication::translate("regwindow", "Register", 0));
    } // retranslateUi

};

namespace Ui {
    class regwindow: public Ui_regwindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_REGWINDOW_H
