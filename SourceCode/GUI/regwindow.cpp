#include "regwindow.h"
#include "ui_regwindow.h"

regwindow::regwindow(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::regwindow)
{
    ui->setupUi(this);
}

regwindow::~regwindow()
{
    delete ui;
}
