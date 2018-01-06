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
#include <exception>

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
    PCList = new QStandardItemModel();
    PCList->clear();
    RPCList = new QStandardItemModel();
    RPCList->clear();
    MeshList = new QStandardItemModel();
    MeshList->clear();
    selectedRaw.clear();
    selectedRegistered.clear();
    selectedMeshed.clear();
    pcViz.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    vtkObject::GlobalWarningDisplayOff();
    pcViz->setBackgroundColor (0.1, 0.1, 0.1);
    pcViz->addCoordinateSystem(1.0);
    pcViz->setCameraPosition(0.0, 0.0, 7, 0.0, 0.0, 0.0);
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

/// @author: Mladen Rakic / Marcio Rockenbach
/// @date: 06-01-2018
/// @version 2.0
///
/// @brief Function to update the list of raw point clouds on the GUI
void MainWindow::updatePCList(QStringList list)
{
    int rawPCs_size = DB->getRawPCs().size();
    int dif = rawPCs_size - PCList->rowCount();
    if (dif >= 0)
        for (int i = 0; i < dif; i++){
            QStandardItem* newitem = new QStandardItem(list[i]);
            newitem->setCheckable(true);
            newitem->setCheckState(Qt::Unchecked);
            PCList->appendRow(newitem);
        }
    ui->pc_list->setModel(PCList);
}

/// @author: Mladen Rakic / Marcio Rockenbach
/// @date: 06-01-2018
/// @version 2.0
///
/// @brief Function to update the list of raw point clouds on the GUI
void MainWindow::updateRegPCList(QStringList list)
{
    int regPCs_size = DB->getRegisteredPCs().size();
    int dif = regPCs_size - RPCList->rowCount();
    if (dif >= 0)
        for (int i = 0; i < dif; i++){
            QStandardItem* newitem = new QStandardItem(list[i]);
            newitem->setCheckable(true);
            newitem->setCheckState(Qt::Unchecked);
            RPCList->appendRow(newitem);
        }
    ui->regpc_list->setModel(RPCList);
}

/// @author: Mladen Rakic / Marcio Rockenbach
/// @date: 06-01-2018
/// @version 2.0
///
/// @brief Function to update the list of raw point clouds on the GUI
void MainWindow::updateMeshList(QStringList list)
{
    int mesh_size = DB->getMeshedPCs().size();
    int dif = mesh_size - MeshList->rowCount();
    if (dif >= 0)
        for (int i = 0; i < dif; i++){
            QStandardItem* newitem = new QStandardItem(list[i]);
            newitem->setCheckable(true);
            newitem->setCheckState(Qt::Unchecked);
            MeshList->appendRow(newitem);
        }
    ui->mesh_list->setModel(MeshList);
}

/// @author: Marcio Rockenbach
/// @date: 06-01-2018
/// @version 1.0
///
/// @brief Function to update the list which contains the index of each checked raw point cloud
void MainWindow::UpdateSelectedRaw(){
    for (int i = 0; i < PCList->rowCount(); i++)
        if (PCList->item(i)->checkState())
            selectedRaw.push_back(i);
}

/// @author: Marcio Rockenbach
/// @date: 06-01-2018
/// @version 1.0
///
/// @brief Function to update the list which contains the index of each checked registered point cloud
void MainWindow::UpdateSelectedReg(){
    for (int i = 0; i < RPCList->rowCount(); i++)
        if (RPCList->item(i)->checkState())
            selectedRegistered.push_back(i);
}

/// @author: Marcio Rockenbach
/// @date: 06-01-2018
/// @version 1.0
///
/// @brief Function to update the list which contains the index of each checked mesh
void MainWindow::UpdateSelectedMesh(){
    for (int i = 0; i < MeshList->rowCount(); i++)
        if (MeshList->item(i)->checkState())
            selectedMeshed.push_back(i);
}

/// @author: Marcio Rockenbach
/// @date: 06-01-2018
/// @version 1.0
///
/// @brief Function to update the VTK Widget to show only the checked point clouds and mesh
void MainWindow::updateDisplay(){
    for (int i = 0; i < PCList->rowCount(); i++)
        if (PCList->item(i)->checkState())
        {
            try{
                pcViz->addPointCloud(DB->getRawPC(i),PCList->item(i)->text().toStdString());
                ui->pcScan->update();
            }
            catch (const std::exception &e)
            {
            }
        }
        else {
            try{
                pcViz->removePointCloud(PCList->item(i)->text().toStdString());
                ui->pcScan->update();
            }
            catch (const std::exception &e)
            {
            }
        }

    for (int i = 0; i < RPCList->rowCount(); i++)
        if (RPCList->item(i)->checkState())
        {
            try{
                pcViz->addPointCloud(DB->getRegisteredPC(i),RPCList->item(i)->text().toStdString());
                ui->pcScan->update();
            }
            catch (const std::exception &e)
            {
            }
        }
        else {
            try{
                pcViz->removePointCloud(RPCList->item(i)->text().toStdString());
                ui->pcScan->update();
            }
            catch (const std::exception &e)
            {
            }
        }

    for (int i = 0; i < MeshList->rowCount(); i++)
        if (MeshList->item(i)->checkState())
        {
            try{
                pcViz->addPolygonMesh(*(DB->getMeshedPC(i)),MeshList->item(i)->text().toStdString());
                ui->pcScan->update();
            }
            catch (const std::exception &e)
            {
            }
        }
        else {
            try{
                pcViz->removePolygonMesh(MeshList->item(i)->text().toStdString());
                ui->pcScan->update();
            }
            catch (const std::exception &e)
            {
            }
        }

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
        MainWindow::updatePCList(qlistPC);
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
    MainWindow::updateRegPCList(qlistPC);
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
    MainWindow::updateMeshList(qlistMesh);
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
        //    foreach (std::string str, selectedRaw)
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


/// @author: Mladen Rakic / Marcio Rockenbach
/// @date: 05-01-2018
/// @version 1.0
///
/// @brief Showing register window on click
void MainWindow::on_mw_register_pc_pushbutton_clicked()
{
    regwindow *reg = new regwindow(this);
    reg->setWindowTitle("MAGMA Project - Register");
    LOG("Register window opened");
    reg->show();
}

void MainWindow::on_mw_generatemesh_pushbutton_clicked()
{

}
void MainWindow::receivedmessage(const QString &arg)
{
    ui->mw_logger_textedit->append(arg);
}

/// @author: Marcio Rockenbach
/// @date: 29-12-2017
/// @version 1.0
///
/// @brief Showing about window on click
void MainWindow::on_actionAbout_triggered()
{
    aboutwindow *about = new aboutwindow(this);
    about->setWindowTitle("MAGMA Project - About");
    about->show();
}

/// @author: Marcio Rockenbach
/// @date: 29-12-2017
/// @version 1.0
///
/// @brief Showing user manual on click
void MainWindow::on_actionUser_manual_triggered()
{
    LOG("User Manual opened");
    QDesktopServices::openUrl(QUrl("user_manual.pdf"));
}

/// @author: Mladen Rakic / Marcio Rockenbach
/// @date: 05-01-2018
/// @version 1.0
///
/// @brief Showing filter window on click
void MainWindow::on_filter_pb_clicked()
{
    filterwindow *filter = new filterwindow(this);
    filter->setWindowTitle("MAGMA Project - Filter");
    LOG("Filter window opened");
    filter->show();
}

void MainWindow::on_pc_list_clicked(const QModelIndex &index)
{
    UpdateSelectedRaw();
    updateDisplay();
}

void MainWindow::on_regpc_list_clicked(const QModelIndex &index)
{
    UpdateSelectedReg();
    updateDisplay();
}

void MainWindow::on_mesh_list_clicked(const QModelIndex &index)
{
    UpdateSelectedMesh();
    updateDisplay();
}
