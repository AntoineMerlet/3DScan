/********************************************************************************
** Form generated from reading UI file 'aboutwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.7.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_ABOUTWINDOW_H
#define UI_ABOUTWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QTextEdit>

QT_BEGIN_NAMESPACE

class Ui_aboutwindow
{
public:
    QTextEdit *show_about;

    void setupUi(QDialog *aboutwindow)
    {
        if (aboutwindow->objectName().isEmpty())
            aboutwindow->setObjectName(QStringLiteral("aboutwindow"));
        aboutwindow->resize(640, 480);
        show_about = new QTextEdit(aboutwindow);
        show_about->setObjectName(QStringLiteral("show_about"));
        show_about->setGeometry(QRect(10, 10, 621, 461));

        retranslateUi(aboutwindow);

        QMetaObject::connectSlotsByName(aboutwindow);
    } // setupUi

    void retranslateUi(QDialog *aboutwindow)
    {
        aboutwindow->setWindowTitle(QApplication::translate("aboutwindow", "Dialog", 0));
    } // retranslateUi

};

namespace Ui {
    class aboutwindow: public Ui_aboutwindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_ABOUTWINDOW_H
