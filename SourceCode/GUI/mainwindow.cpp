#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "IO/kinect2_grabber.h"
#include <QFileDialog>
#include <QStringList>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_actionNew_scan_triggered()
{
    QStringList files = QFileDialog::getOpenFileNames(this, tr("Select files to open"),"", tr(""));
}
