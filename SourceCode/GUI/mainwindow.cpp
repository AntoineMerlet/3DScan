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
#include <QStandardItem>
#include <logger.h>
#include <GUI/aboutwindow.h>
#include <QDesktopServices>
#include <QListWidget>
#include <QListWidgetItem>
#include <QList>
#include <QString>
#include <QAbstractListModel>
#include <QLabel>
#include <QPixmap>

using namespace std;


/// @author: Antoine Merlet
/// @date: 28-12-2017
/// @version 1.0
///
/// @brief Constructor of main window. mainwindow is holding the pointer to the DataBase, and inits the PCl vizualiser
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    QPixmap logo ("logo_small.jpg");
    ui->mylogo->setPixmap(logo);
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


/// @author: Antoine Merlet
/// @date: 28-12-2017
/// @version 1.0
///
/// @brief Showing scan window on click
void MainWindow::on_actionNew_scan_triggered()
{
    //...
    scanwindow *scan = new scanwindow(this);
    scan->setWindowTitle("MAGMA Project - New Scan");
    scan->show();
}


/// @author: Mladen Rakic
/// @date: 28-12-2017
/// @version 1.0
///
/// @brief Function to update the list of raw point clouds on the GUI
void MainWindow::updatePCList()
{
    PCList = new QStandardItemModel();
    PCList->clear();
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> rawPCs = DB->getRawPCs();
    for (int i = 1; i <= rawPCs.size(); i++){
        stringstream ss;
        ss << "Pointcloud " << i;
        string PCName = ss.str();
        QStandardItem* Items = new QStandardItem(QString::fromStdString(PCName));
        Items->setCheckable(true);
        Items->setCheckState(Qt::Unchecked);
        PCList->appendRow(Items);
    }
    ui->pc_list->setModel(PCList);
}

/// @author: Mladen Rakic
/// @date: 28-12-2017
/// @version 1.0
///
/// @brief Function to update the list of raw point clouds on the GUI
void MainWindow::updateRegPCList()
{
    RPCList = new QStandardItemModel();
    RPCList->clear();
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> regPCs = DB->getRegisteredPCs();
    for (int i = 1; i <= regPCs.size(); i++){
        stringstream ss;
        ss << "Registered Pointcloud " << i;
        string PCName = ss.str();
        QStandardItem* Items = new QStandardItem(QString::fromStdString(PCName));
        Items->setCheckable(true);
        Items->setCheckState(Qt::Unchecked);
        RPCList->appendRow(Items);
    }
    ui->regpc_list->setModel(RPCList);
}

/// @author: Mladen Rakic
/// @date: 28-12-2017
/// @version 1.0
///
/// @brief Function to update the list of raw point clouds on the GUI
void MainWindow::updateMeshList()
{
    MeshList = new QStandardItemModel();
    MeshList->clear();
    std::vector<pcl::PolygonMesh::Ptr> mesh = DB->getMeshedPCs();
    for (int i = 1; i <= mesh.size(); i++){
        stringstream ss;
        ss << "Mesh " << i;
        string PCName = ss.str();
        QStandardItem* Items = new QStandardItem(QString::fromStdString(PCName));
        Items->setCheckable(true);
        Items->setCheckState(Qt::Unchecked);
        MeshList->appendRow(Items);
    }
    ui->mesh_list->setModel(MeshList);
}


/// @author: Antoine Merlet
/// @date: 28-12-2017
/// @version 1.0
///
/// @brief Opens a file dialog for the user to select one or several Point Clouds to load. These point cloouds are stored in DataBase
void MainWindow::on_actionImport_point_clouds_triggered()
{
    QStringList qlistPC = QFileDialog::getOpenFileNames(this, QString("Import Point Clouds"), QString(""), QString("Point Cloud (*.pcd *.ply)"));
    if (qlistPC.size() > 0)
    {
        rawPC2DB(qlistPC,DB);
        MainWindow::updatePCList();

    }
}


/// @author: Antoine Merlet
/// @date: 28-12-2017
/// @version 1.0
///
/// @brief Opens a file dialog for the user to select one or several Point Clouds to load. These point cloouds are stored in DataBase
void MainWindow::on_actionImport_registered_PC_triggered()
{
    QStringList qlistPC = QFileDialog::getOpenFileNames(this, QString("Import Point Clouds"), QString(""), QString("Point Cloud (*.pcd *.ply)"));
    if (qlistPC.size() > 0)
        registeredPC2DB(qlistPC,DB);
    MainWindow::updateRegPCList();
}

/// @author: Antoine Merlet
/// @date: 28-12-2017
/// @version 1.0
///
/// @brief Opens a file dialog for the user to select one or several meshs to load. These meshs are stored in DataBase
void MainWindow::on_actionImport_mesh_triggered()
{
    QStringList qlistMesh = QFileDialog::getOpenFileNames(this, QString("Import Mesh"), QString(""), QString("Mesh (*.stl *.vtk)"));
    if (qlistMesh.size() > 0)
        meshedPC2DB(qlistMesh,DB);
    MainWindow::updateMeshList();
}


/// @author: Antoine Merlet
/// @date: 28-12-2017
/// @version 0.1
///
/// @brief Opens a file dialog for the user to select a folder to save Point Clouds.
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


/// @author: Antoine Merlet
/// @date: 28-12-2017
/// @version 0.1
///
/// @brief Opens a file dialog for the user to select a folder to save Point Clouds.
void MainWindow::on_actionExport_registered_PC_triggered()
{
    //...
}


/// @author: Antoine Merlet
/// @date: 28-12-2017
/// @version 0.1
///
/// @brief Opens a file dialog for the user to select a folder to save meshs.
void MainWindow::on_actionExport_mesh_triggered()
{
    //...
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

void MainWindow::on_actionAbout_triggered()
{
    aboutwindow *about = new aboutwindow(this);
    about->setWindowTitle("MAGMA Project - About");
    about->show();
}

void MainWindow::on_actionUser_manual_triggered()
{
    LOG("User Manual");
    QDesktopServices::openUrl(QUrl("user_manual.pdf"));
}
