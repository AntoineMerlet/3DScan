#include "aboutwindow.h"
#include "ui_aboutwindow.h"
#include <QFile>
#include <QTextStream>
#include <IO/logger.h>

/// @author: Marcio Rockenbach
/// @date: 28-12-2017
/// @version 1.0
///
/// @brief Displays the About section from a html file.
aboutwindow::aboutwindow(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::aboutwindow)
{
    ui->setupUi(this);
    QFile file("about.html");
    file.open(QFile::ReadOnly | QFile::Text);
    QTextStream stream(&file);
    ui->show_about->setHtml(stream.readAll());       
    LOG("About section opened");
}

aboutwindow::~aboutwindow()
{
    delete ui;
}
