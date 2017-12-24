#include "scanwindow.h"
#include "Storage/database.h"
#include "ui_scanwindow.h"

scanwindow::scanwindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::scanwindow)
{
    ui->setupUi(this);
}

scanwindow::~scanwindow()
{
    delete ui;
}


/// @author: Mladen Rakic
/// @date: 24-12-2017
/// @version 1.0
///
/// @brief Function used to choose vertical acquisition mode.
/// @param bool indicator of the button pressed
void scanwindow::on_sw_verticalacq_radiobutton_clicked(bool checked)
{
    ui->sw_horizontalacq_radiobutton->setChecked(false);
    //...
}

void scanwindow::on_sw_horizontalacq_radiobutton_clicked(bool checked)
{
    ui->sw_verticalacq_radiobutton->setChecked(false);
    //...
}

void scanwindow::on_sw_xmin_horslider_actionTriggered(int action)
{
    dataclass->set_xmin(action);
}

void scanwindow::on_sw_xmax_horslider_actionTriggered(int action)
{
    dataclass->set_xmax(action);
}

void scanwindow::on_sw_ymin_horslider_actionTriggered(int action)
{
    dataclass->set_ymin(action);
}

void scanwindow::on_sw_ymax_horslider_actionTriggered(int action)
{
    dataclass->set_ymax(action);
}

void scanwindow::on_sw_zmin_horslider_actionTriggered(int action)
{
    dataclass->set_xmin(action);
}

void scanwindow::on_sw_zmax_horslider_actionTriggered(int action)
{
    dataclass->set_zmax(action);
}
