#include "filterwindow.h"
#include "ui_filterwindow.h"

filterwindow::filterwindow(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::filterwindow)
{
    ui->setupUi(this);
}

filterwindow::~filterwindow()
{
    delete ui;
}
