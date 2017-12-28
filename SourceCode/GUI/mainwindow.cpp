#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "guitools.h"
#include "IO/iotools.h"
#include "scanwindow.h"

#include "IO/kinect2_grabber.h"
#include <QFileDialog>
#include <QStringList>
#include <string>
#include <QDebug>
#include <QDockWidget>
<<<<<<< HEAD
#include <QStandardItem>
=======
#include <logger.h>
>>>>>>> 57152475ba034a8192cfe6a8cb0a1686a6000800

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
    scanwindow *scan = new scanwindow(this);
    scan->show();
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


/// @author: Mladen Rakic
/// @date: 28-12-2017
/// @version 1.0
///
/// @brief Function to update the list of raw point clouds on the GUI
void MainWindow::updatePCList()
{

    PCList->clear();
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> rawPCs = DB::getRawPCs();
    for (int i = 1; i <= rawPCs.size(); i++){
        stringstream ss;
        ss << "Pointcloud " << i;
        string PCName = ss.str();
        QStandardItem* Items = new QStandardItem(QString::fromStdString(PCName));
        PCList->appendRow(Items);
    }

    ui->pc_list->setModel(PCList);

}

//Load point clouds
void MainWindow::on_actionImport_point_clouds_triggered()
{
    QStringList qlistPC = QFileDialog::getOpenFileNames(this, QString("Import Point Clouds"), QString(""), QString("Point Cloud (*.pcd *.ply)"));
    if (qlistPC.size() > 0)
    {
        rawPC2DB(qlistPC,DB);

    }
}

//Load registered point cloud
void MainWindow::on_actionImport_registered_PC_triggered()
{
    QStringList qlistPC = QFileDialog::getOpenFileNames(this, QString("Import Point Clouds"), QString(""), QString("Point Cloud (*.pcd *.ply)"));
    if (qlistPC.size() > 0)
        registeredPC2DB(qlistPC,DB);
}

//Load mesh
void MainWindow::on_actionImport_mesh_triggered()
{
    QStringList qlistMesh = QFileDialog::getOpenFileNames(this, QString("Import Mesh"), QString(""), QString("Mesh (*.stl *.vtk)"));
    if (qlistMesh.size() > 0)
        meshedPC2DB(qlistMesh,DB);
}

//Save acquired point clouds
void MainWindow::on_actionExport_point_clouds_triggered()
{
    QString qdir = QFileDialog::getExistingDirectory(this, QString("Export point cloud"),QString(""), QFileDialog::ShowDirsOnly);
    if (qdir != "")
    {
        foreach (std::string str, selectedRaw)
        {

        }
    }
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


void MainWindow::on_mw_register_pc_pushbutton_clicked()
{

}

void MainWindow::on_mw_generatemesh_pushbutton_clicked()
{

}
void MainWindow::receivedmessage(const QString &arg)
{
    ui->mw_logger_textedit->append(arg);
}
