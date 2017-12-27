#include "scanwindow.h"
#include "Storage/database.h"
#include "ui_scanwindow.h"
#include <string>

using namespace std;

scanwindow::scanwindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::scanwindow)
{
    ui->setupUi(this);
}

scanwindow::~scanwindow()
{
    QApplication::exit();
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

/// @author: Mladen Rakic
/// @date: 24-12-2017
/// @version 1.0
///
/// @brief Function used to choose horizontal acquisition mode.
/// @param bool indicator of the button pressed
void scanwindow::on_sw_horizontalacq_radiobutton_clicked(bool checked)
{
    ui->sw_verticalacq_radiobutton->setChecked(false);
    //...
}

//Modify the spatial coordinates
void scanwindow::on_sw_xmin_horslider_actionTriggered(int action)
{
   //dataclass->set_xmin(action);
}

void scanwindow::on_sw_xmax_horslider_actionTriggered(int action)
{
    //dataclass->set_xmax(action);
}

void scanwindow::on_sw_ymin_horslider_actionTriggered(int action)
{
    //dataclass->set_ymin(action);
}

void scanwindow::on_sw_ymax_horslider_actionTriggered(int action)
{
    //dataclass->set_ymax(action);
}

void scanwindow::on_sw_zmin_horslider_actionTriggered(int action)
{
    //dataclass->set_zmin(action);
}

void scanwindow::on_sw_zmax_horslider_actionTriggered(int action)
{
    //dataclass->set_zmax(action);
}

/// @author: Mladen Rakic
/// @date: 26-12-2017
/// @version 1.0
///
/// @brief Function to display text from logger file in scan window.
/// @param filename Name of the text file to be read
void scanwindow::readfile(std::string filename){
//    QFile file(filename);
//    if(!file.exists()){
//        qDebug() << "File not found! "<<filename;
//    }else{
//        qDebug() << filename<<" File found";
//    }
//    std::string line;
//    ui->sw_logger_textedit->clear();
//    if (file.open(QIODevice::ReadOnly | QIODevice::Text)){
//        QTextStream stream(&file);
//        while (!stream.atEnd()){
//            line = stream.readLine();
//            ui->sw_logger_textedit->setText(ui->sw_logger_textedit->toPlainText()+line+"\n");
//            qDebug() << "Line: "<<line;
//        }
//    }
//    file.close();
}

void scanwindow::on_sw_horizontalacq_radiobutton_clicked()
{

}
