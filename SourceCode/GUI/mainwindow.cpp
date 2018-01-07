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
#include <QStandardItem>
#include <IO/logger.h>
#include <GUI/aboutwindow.h>
#include <QDesktopServices>
#include <QListWidget>
#include <QListWidgetItem>
#include <QList>
#include <QString>
#include <QAbstractListModel>
#include <QLabel>
#include <QPixmap>
#include <GUI/filterwindow.h>
#include "GUI/regwindow.h"
#include <exception>
#include "Core/filtering.h"
#include "Core/registration.h"
#include <pcl/registration/correspondence_estimation.h>
#include "Core/mathtools.h"
#include <cmath>
#include <pcl/filters/random_sample.h>
#include <QCloseEvent>

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
    FW = new filterwindow(this);
    FW->close();
    RW = new regwindow(this);
    RW->close();
    vtkObject::GlobalWarningDisplayOff();
    pcViz.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    ui->pcScan->SetRenderWindow(pcViz->getRenderWindow() );
    pcViz->setupInteractor(ui->pcScan->GetInteractor(),ui->pcScan->GetRenderWindow());
    pcViz->setBackgroundColor (0.1, 0.1, 0.1);
    pcViz->addCoordinateSystem(1.0);
    pcViz->setCameraPosition(0.0, 0.0, 7, 0.0, 0.0, 0.0);
}

MainWindow::~MainWindow()
{
    delete DB;
    delete scan;
    delete FW;
    delete RW;
    delete PCList;
    delete RPCList;
    delete MeshList;
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
    scan = new scanwindow(this);
    QObject::connect(scan,SIGNAL(send_unhide()),this,SLOT(unhidemain()));
    scan->setWindowTitle("MAGMA Project - New Scan");
    scan->move(700,100);
    scan->show();
    MainWindow::setVisible(false);
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
            QString str = list[i];
            str = str.section('/',-1);
            str = str.section('.',0,0);
            QStandardItem* newitem = new QStandardItem(str);
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
/// @brief Function to update the list of registered point clouds on the GUI
void MainWindow::updateRegPCList(QStringList list)
{
    int regPCs_size = DB->getRegisteredPCs().size();
    int dif = regPCs_size - RPCList->rowCount();
    if (dif >= 0)
        for (int i = 0; i < dif; i++){
            QString str = list[i];
            str = str.section('/',-1);
            str = str.section('.',0,0);
            QStandardItem* newitem = new QStandardItem(str);
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
/// @brief Function to update the list of meshes on the GUI
void MainWindow::updateMeshList(QStringList list)
{
    int mesh_size = DB->getMeshedPCs().size();
    int dif = mesh_size - MeshList->rowCount();
    if (dif >= 0)
        for (int i = 0; i < dif; i++){
            QString str = list[i];
            str = str.section('/',-1);
            str = str.section('.',0,0);
            QStandardItem* newitem = new QStandardItem(str);
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
/// @brief Function to update the list which contains the index of each selected raw point cloud in the GUI
void MainWindow::UpdateSelectedRaw(){
    for (int i = 0; i < PCList->rowCount(); i++)
        if (PCList->item(i)->checkState())
            selectedRaw.insert(i);
        else selectedRaw.erase(i);
}

/// @author: Marcio Rockenbach
/// @date: 06-01-2018
/// @version 1.0
///
/// @brief Function to update the list which contains the index of each selected registered point cloud in the GUI
void MainWindow::UpdateSelectedReg(){
    for (int i = 0; i < RPCList->rowCount(); i++)
        if (RPCList->item(i)->checkState())
            selectedRegistered.insert(i);
        else selectedRegistered.erase(i);
}

/// @author: Marcio Rockenbach
/// @date: 06-01-2018
/// @version 1.0
///
/// @brief Function to update the list which contains the index of each selected mesh in the GUI
void MainWindow::UpdateSelectedMesh(){
    for (int i = 0; i < MeshList->rowCount(); i++)
        if (MeshList->item(i)->checkState())
            selectedMeshed.insert(i);
        else selectedMeshed.erase(i);
}

/// @author: Marcio Rockenbach
/// @date: 06-01-2018
/// @version 1.0
///
/// @brief Function to update the VTK Widget to show only the selected point clouds and mesh
void MainWindow::updateDisplay(){
    for (int i = 0; i < PCList->rowCount(); i++) // Displaying selected raw point clouds
        if (PCList->item(i)->checkState())
        {
            try{
                pcViz->addPointCloud(DB->getRawPC(i),PCList->item(i)->text().toStdString());
                ui->pcScan->update();
            }
            catch (const std::exception &)
            {
            }
        }
        else {
            try{
                pcViz->removePointCloud(PCList->item(i)->text().toStdString());
                ui->pcScan->update();
            }
            catch (const std::exception &)
            {
            }
        }

    for (int i = 0; i < RPCList->rowCount(); i++) // Displaying selected registered point clouds
        if (RPCList->item(i)->checkState())
        {
            try{
                pcViz->addPointCloud(DB->getRegisteredPC(i),RPCList->item(i)->text().toStdString());
                ui->pcScan->update();
            }
            catch (const std::exception &)
            {
            }
        }
        else {
            try{
                pcViz->removePointCloud(RPCList->item(i)->text().toStdString());
                ui->pcScan->update();
            }
            catch (const std::exception &)
            {
            }
        }

    for (int i = 0; i < MeshList->rowCount(); i++) // Displaying selected mesh
        if (MeshList->item(i)->checkState())
        {
            try{
                pcViz->addPolygonMesh(*(DB->getMeshedPC(i)),MeshList->item(i)->text().toStdString());
                ui->pcScan->update();
            }
            catch (const std::exception &)
            {
            }
        }
        else {
            try{
                pcViz->removePolygonMesh(MeshList->item(i)->text().toStdString());
                ui->pcScan->update();
            }
            catch (const std::exception &)
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
/// @brief Showing filter window on click
void MainWindow::on_filter_pb_clicked()
{

    LOG("Filter window opened");
    FW->show();
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
/// @brief Shows about window on click
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
/// @brief Shows user manual on click
void MainWindow::on_actionUser_manual_triggered()
{
    LOG("User Manual opened");
    QDesktopServices::openUrl(QUrl("user_manual.pdf"));
}


/// @author: Marcio Rockenbach
/// @date: 06-01-2018
/// @version 1.0
///
/// @brief Updates the display and the set of selected raw point clouds whenever a user clicks on the list of loaded raw point clouds on the GUI
void MainWindow::on_pc_list_clicked(const QModelIndex &)
{
    UpdateSelectedRaw();
    updateDisplay();
    LOG("Display updated");
}

/// @author: Marcio Rockenbach
/// @date: 06-01-2018
/// @version 1.0
///
/// @brief Updates the display and the set of selected registered point clouds whenever a user clicks on the List of loaded registered point clouds on the GUI
void MainWindow::on_regpc_list_clicked(const QModelIndex &)
{
    UpdateSelectedReg();
    updateDisplay();
    LOG("Display updated");
}

/// @author: Marcio Rockenbach
/// @date: 06-01-2018
/// @version 1.0
///
/// @brief Updates the display and the set of selected Mesh whenever a user clicks on the list of loaded meshes on the GUI
void MainWindow::on_mesh_list_clicked(const QModelIndex &)
{
    UpdateSelectedMesh();
    updateDisplay();
    LOG("Display updated");
}

void MainWindow::updatef() {
    std::set<int>::iterator it = selectedRaw.begin();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr currentPC;
    LOG("Processing " + std::to_string(selectedRaw.size()) +" point clouds");
    for(it; it != selectedRaw.end(); it++)
    {
        currentPC = DB->getRawPC(*it);
        if (!currentPC->isOrganized())
        {
            currentPC->width = 640;
            currentPC->height = 480;
            currentPC->points.resize(currentPC->width * currentPC->height);
        }
        if (FW->bilateralfilt.checked)
            *currentPC = *Core::bilateralFilter(currentPC, FW->bilateralfilt.sigmaR, FW->bilateralfilt.sigmaS);
        if (FW->voxelgridfilt.checked)
            *currentPC = *Core::downsample(currentPC,FW->voxelgridfilt.x,FW->voxelgridfilt.y,FW->voxelgridfilt.z);
        if (FW->medianfilt.checked)
            *currentPC = *Core::medianFilter(currentPC,FW->medianfilt.windowsize, FW->medianfilt.maxmovement);
        if (FW->randomfilt.checked)
            *currentPC = *Core::randomSample(currentPC,100);
        if (FW->normalfilt.checked)
            *currentPC = *Core::normalSample(currentPC,FW->normalfilt.order,FW->normalfilt.nofbins, FW->paramsfilt.maxdepth, FW->paramsfilt.smoothsize);
        if (FW->covarfilt.checked)
            *currentPC = *Core::covarianceSample(currentPC, FW->covarfilt.order,FW->paramsfilt.maxdepth, FW->paramsfilt.smoothsize);
        DB->replaceRawPC(currentPC,*it);
        pcViz->updatePointCloud(currentPC,PCList->item(*it)->text().toStdString());
    }

}

void MainWindow::updater() {
    if (selectedRaw.size() > 1)
    {
        LOG("Processing " + std::to_string(selectedRaw.size()) +" point clouds");

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr src, target;
        pcl::PointCloud<pcl::PointNormal>::Ptr srcN, targetN;
        for(int i = 1; i < selectedRaw.size(); i++)
        {
            src = DB->getRawPC(i-1);
            target = DB->getRawPC(i);

            srcN = Core::getNormalPoints(src,0.01f,50);
            targetN = Core::getNormalPoints(target,0.01f,50);

            pcl::PointCloud<pcl::PointNormal>::Ptr srcNS(new pcl::PointCloud<pcl::PointNormal>), targetNS(new pcl::PointCloud<pcl::PointNormal>);
            pcl::RandomSample<pcl::PointNormal> randsample;
            randsample.setSample(srcN->size() / 100);
            randsample.setInputCloud(srcN);
            randsample.filter(*srcNS);
            randsample.setInputCloud(targetN);
            randsample.filter(*targetNS);

            LOG("Processing " + std::to_string(srcNS->size()) +" point clouds"+ std::to_string(targetNS->size()) );
            LOG("test");
            pcl::Correspondences corresp = Core::fullCorresp(srcNS,targetNS,1000,0.2,1,acos (90.0 * M_PI / 180.0),0.05);
        }
    }
}

void MainWindow::closeEvent(QCloseEvent *e)
{
    foreach (QWidget *widget, QApplication::topLevelWidgets()) {
        if (widget != this) { // avoid recursion.
            widget->close();
        }
    }
    LOG("Main window closed");
    e->accept();
}

void MainWindow::unhidemain(){
    MainWindow::setVisible(true);
}
