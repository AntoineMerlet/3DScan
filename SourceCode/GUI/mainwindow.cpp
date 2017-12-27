#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "IO/kinect2_grabber.h"
#include <QFileDialog>
#include <QStringList>
#include <string>
#include <QDebug>
#include <QDesktopServices>
#include <QUrl>

using namespace std;

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
    //...
}

/// @author: Mladen Rakic
/// @date: 26-12-2017
/// @version 1.0
///
/// @brief Function to display text from logger file in main window.
/// @param filename Name of the text file to be read
void MainWindow::readfile(std::string filename){
    QFile file(filename);
    if(!file.exists()){
        qDebug() << "File not found! " << filename;
    }else{
        qDebug() << filename << " File found";
    }
    std::string line;
    ui->mw_logger_textedit->clear();
    if (file.open(QIODevice::ReadOnly | QIODevice::Text)){
        QTextStream stream(&file);
        while (!stream.atEnd()){
            line = stream.readLine();
            ui->mw_logger_textedit->setText(ui->mw_logger_textedit->toPlainText()+line+"\n");
            qDebug() << "Line: "<<line;
        }
    }
    file.close();
}

//Main window destructor
MainWindow::~MainWindow()
{
    QApplication::exit();
    delete ui;
}

/// @author: Mladen Rakic
/// @date: 27-12-2017
/// @version 1.0
///
/// @brief Function to import point clouds
void MainWindow::on_actionImport_point_clouds_triggered()
{
    QStringList pointCloudFileNamesList = QFileDialog::getOpenFileNames(this, QString("Import point clouds"), QString(""), QString("Point Cloud (*.pcd *.ply)"));
    if(pointCloudFileNamesList.size() !=0){
        for(int i = 0; i < pointCloudFileNamesList.size(); i++){
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudPtr (new pcl::PointCloud<pcl::PointXYZRGB>());
            //Load ply files
            if(pointCloudFileNamesList[i].endsWith(".ply")){
                pcl::io::loadPLYFile(pointCloudFileNamesList[i].toStdString(), *pointCloudPtr);
                database::mf_StaticAddPointCloud(pointCloudPtr);
                //emit mf_SignalDatabasePointCloudUpdated();
            }
            //Load pcd files
            else if(pointCloudFileNamesList[i].endsWith(".pcd")){
                pcl::io::loadPCDFile(pointCloudFileNamesList[i].toStdString(), *pointCloudPtr);
                database::getRawPCs(pointCloudPtr);
                //emit mf_SignalDatabasePointCloudUpdated();
            }

        }
    }
}

//Load registered point cloud
void MainWindow::on_actionImport_registered_pc_triggered()
{
    //...
}

/// @author: Mladen Rakic
/// @date: 27-12-2017
/// @version 1.0
///
/// @brief Function to import mesh
void MainWindow::on_actionImport_mesh_triggered()
{
    QStringList meshFileNamesList = QFileDialog::getOpenFileNames(this, QString("Import mesh"), QString(""), QString("Mesh (*.vtk *.stl)"));
    if(meshFileNamesList.size() !=0){
        for(int i = 0; i < meshFileNamesList.size(); i++){
            pcl::PolygonMesh::Ptr meshPtr (new pcl::PolygonMesh());

            if(meshFileNamesList[i].endsWith(".stl")){
                pcl::io::loadPolygonFileSTL(meshFileNamesList[i].toStdString(), *meshPtr);
                database::getMeshedPCs(meshPtr);
                //emit mf_SignalDatabaseMeshUpdated();
            }

            else if(meshFileNamesList[i].endsWith(".vtk")){
                pcl::io::loadPolygonFileVTK(meshFileNamesList[i].toStdString(), *meshPtr);
                database::getMeshedPCs(meshPtr);
                //emit mf_SignalDatabaseMeshUpdated();
            }
        }
    }
}

/// @author: Mladen Rakic
/// @date: 27-12-2017
/// @version 1.0
///
/// @brief Function to export point clouds
void MainWindow::on_actionExport_point_clouds_triggered()
{
        QString directoryName = QFileDialog::getExistingDirectory(this, QString("Export point cloud"),QString(""), QFileDialog::ShowDirsOnly);
        if(directoryName != ""){
            QString filePath;
            QListWidgetItem* item;

            //Save pointclouds selected in pointcloud tab
            for(int i = 0, len = MainWindow->mw_pc_tab->count(); i < len; i++)
            {
                item = MainWindow->mw_pc_tab->item(i);
                if(item->checkState() == Qt::Checked)
                {
                    filePath = directoryName + "/" + item->text() + ".pcd";
                    qDebug() << filePath << " : pointcloud size - " << database::rawPCs[i]->points.size();
                    pcl::io::savePCDFile(filePath.toStdString(), *(database::rawPCs[i]));
                }
            }

        }

}

/// @author: Mladen Rakic
/// @date: 27-12-2017
/// @version 1.0
///
/// @brief Function to export registered point clouds
void MainWindow::on_actionExport_registered_pc_triggered()
{
    QString directoryName = QFileDialog::getExistingDirectory(this, QString("Export point cloud"),QString(""), QFileDialog::ShowDirsOnly);
    if(directoryName != ""){
        QString filePath;
        QListWidgetItem* item;

        //Save pointclouds selected in registered pointcloud tab
        for(int i = 0, len = MainWindow->mw_registeredpc_tab->count(); i < len; i++)
        {
            item = MainWindow->mw_registeredpc_tab->item(i);
            if(item->checkState() == Qt::Checked)
            {
                filePath = directoryName + "/" + item->text() + ".pcd";
                qDebug() << filePath;
                pcl::io::savePCDFile(filePath.toStdString(), *(database::registeredPCs[i]));
            }
        }
    }
}

//Save generated mesh
void MainWindow::on_actionExport_mesh_triggered()
{
    //...
}

/// @author: Mladen Rakic
/// @date: 27-12-2017
/// @version 1.0
///
/// @brief Function to open the "about" file
void MainWindow::on_actionAbout_triggered()
{
    //still have to figure out how make path independent from the computer used
    QDesktopServices::openUrl(QUrl("file:///C:/Users/Mladen/Desktop/MAIA%20uB/Software%20Engineering/Project/3DScan/SourceCode/about.html"));
}

/// @author: Mladen Rakic
/// @date: 26-12-2017
/// @version 1.0
///
/// @brief Function to set up the QVTK Widget in the main window
void MainWindow::resetVtk()
{
    // Set up the QVTK widget
    viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    ui->mw_qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
    viewer->setupInteractor (ui->mw_qvtkWidget->GetInteractor (), ui->mw_qvtkWidget->GetRenderWindow ());
    ui->mw_qvtkWidget->update ();

    pcl::visualization::PointCloudColorHandlerCustom<PointType> colorPt (database->m_cloud, 0, 123, 100);
    viewer->addPointCloud (database->m_cloud, colorPt,  "cloud");
    viewer->resetCamera ();
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
    ui->mw_qvtkWidget->update ();

}

