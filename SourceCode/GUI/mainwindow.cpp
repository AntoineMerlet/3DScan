#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "guitools.h"
#include "IO/iotools.h"

#include "IO/kinect2_grabber.h"
#include <QFileDialog>
#include <QStringList>
#include <string>
#include <QDebug>
#include <QDockWidget>

using namespace std;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    DB = new DataBase();

    pcViz.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    pcViz->setBackgroundColor (0.1, 0.1, 0.1);
    pcViz->addCoordinateSystem(1.0);
    ui->pcScan->SetRenderWindow(pcViz->getRenderWindow() );
    pcViz->setupInteractor(ui->pcScan->GetInteractor(),ui->pcScan->GetRenderWindow());
}

MainWindow::~MainWindow()
{
    delete DB;
    QApplication::exit();
    delete ui;
}

void MainWindow::on_actionNew_scan_triggered()
{
    //...
}

/// @author: Mladen Rakic
/// @date: 26-12-2017
/// @version 1.0
///
/// @brief Function to display text from logger file in main window.
/// @param filename Name of the text file to be read
void MainWindow::readfile(std::string filename){
//    QFile file(filename);
//    if(!file.exists()){
//        qDebug() << "File not found! " << filename;
//    }else{
//        qDebug() << filename << " File found";
//    }
//    std::string line;
//    ui->mw_logger_textedit->clear();
//    if (file.open(QIODevice::ReadOnly | QIODevice::Text)){
//        QTextStream stream(&file);
//        while (!stream.atEnd()){
//            line = stream.readLine();
//            ui->mw_logger_textedit->setText(ui->mw_logger_textedit->toPlainText()+line+"\n");
//            qDebug() << "Line: "<<line;
//        }
//    }
//    file.close();
}

//Load point clouds
void MainWindow::on_actionImport_point_clouds_triggered()
{
    QStringList qlistPC = QFileDialog::getOpenFileNames(this, QString("Import point clouds"), QString(""), QString("Point Cloud (*.pcd *.ply)"));
    if (qlistPC.size() > 0)
    {
        std::vector<std::string> listPC = QStringList2StdStringVec(qlistPC);
        DB->addRawPC(IO::loadPCD(listPC[0]));
    }
    pcViz->addPointCloud(DB->getRawPC(0), "toto");
}

//Load registered point cloud
void MainWindow::on_actionImport_registered_PC_triggered()
{
    //...
}

//Load mesh
void MainWindow::on_actionImport_mesh_triggered()
{
    //...
}

//Save acquired point clouds
void MainWindow::on_actionExport_point_clouds_triggered()
{
    //...
}

//Save registered point cloud
void MainWindow::on_actionExport_registered_PC_triggered()
{
    //...
}

//Save generated mesh
void MainWindow::on_actionExport_mesh_triggered()
{
    //...
}

/// @author: Mladen Rakic
/// @date: 26-12-2017
/// @version 1.0
///
/// @brief Function to set up the QVTK Widget in the main window
void MainWindow::resetVtk()
{
//    // Set up the QVTK widget
//    viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
//    ui->mw_qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
//    viewer->setupInteractor (ui->mw_qvtkWidget->GetInteractor (), ui->mw_qvtkWidget->GetRenderWindow ());
//    ui->mw_qvtkWidget->update ();

//    pcl::visualization::PointCloudColorHandlerCustom<PointType> colorPt (database->m_cloud, 0, 123, 100);
//    viewer->addPointCloud (database->m_cloud, colorPt,  "cloud");
//    viewer->resetCamera ();
//    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
//    ui->mw_qvtkWidget->update ();

}

