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
#include <pcl/filters/random_sample.h>
#include <QCloseEvent>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/icp.h>



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
    scan = new scanwindow(this);
    QObject::connect(scan,SIGNAL(send_unhide()),this,SLOT(unhidemain()));
    FW = new filterwindow(this);
    FW->close();
    RW = new regwindow(this);
    RW->close();
    vtkObject::GlobalWarningDisplayOff();
    pcViz.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    ui->pcScan->SetRenderWindow(pcViz->getRenderWindow() );
    pcViz->setupInteractor(ui->pcScan->GetInteractor(),ui->pcScan->GetRenderWindow());
    pcViz->setBackgroundColor (0.1, 0.1, 0.1);
    pcViz->addCoordinateSystem(1.0, -1.0, -0.5, -0.5);
    pcViz->setShowFPS(false);
    pcViz->setCameraPosition(0.0, 0.0, -5, 0.0, 0.0, 0.0);
}

MainWindow::~MainWindow()
{
    delete DB;
    delete FW;
    delete RW;
    delete scan;
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
    scan->setWindowTitle("MAGMA Project - New Scan");
    scan->move(700,100);
    MainWindow::setVisible(false);
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
        std::set<int>::iterator it = selectedRaw.begin();
            for(it; it != selectedRaw.end(); it++)
            {
                LOG(qdir.toStdString() + PCList->item(*it)->text().toStdString());
                IO::savePLY(DB->getRawPC(*it),qdir.toStdString() + "/" + PCList->item(*it)->text().toStdString() + ".ply");
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
    QString qdir = QFileDialog::getExistingDirectory(this, QString("Export point cloud"),QString(""), QFileDialog::ShowDirsOnly);
    if (qdir != "")
    {
        std::set<int>::iterator it = selectedRegistered.begin();
            for(it; it != selectedRegistered.end(); it++)
            {
                IO::savePLY(DB->getRegisteredPC(*it),qdir.toStdString() + "/" + RPCList->item(*it)->text().toStdString() + ".ply");
            }

    }
}


/// @author: Antoine Merlet
/// @date: 28-12-2017
/// @version 0.1
///
/// @brief Opens a file dialog for the user to select a folder to save meshs.
void MainWindow::on_actionExport_mesh_triggered()
{
    LOG("not implemented yet");
}


/// @author: Mladen Rakic / Marcio Rockenbach
/// @date: 05-01-2018
/// @version 1.0
///
/// @brief Showing filter window on click
void MainWindow::on_filter_pb_clicked()
{
    FW->setWindowTitle("MAGMA Project - Filter");
    FW->show();
}


/// @author: Mladen Rakic / Marcio Rockenbach
/// @date: 05-01-2018
/// @version 1.0
///
/// @brief Showing register window on click
void MainWindow::on_mw_register_pc_pushbutton_clicked()
{
    RW->setWindowTitle("MAGMA Project - Register");
    RW->show();
}

void MainWindow::on_mw_generatemesh_pushbutton_clicked()
{
    LOG("not implemented yet");
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
            *currentPC = *Core::randomSample(currentPC,FW->randomfilt.order);
        if (FW->normalfilt.checked)
            *currentPC = *Core::normalSample(currentPC,FW->normalfilt.order,FW->normalfilt.nofbins, FW->paramsfilt.radius);
        if (FW->covarfilt.checked)
            *currentPC = *Core::covarianceSample(currentPC, FW->covarfilt.order,FW->paramsfilt.radius);
        DB->replaceRawPC(currentPC,*it);
        pcViz->updatePointCloud(currentPC,PCList->item(*it)->text().toStdString());
    }

}

void MainWindow::updater() {
    if (selectedRaw.size() > 1)
    {
        LOG("Processing " + std::to_string(selectedRaw.size()) +" point clouds");
        std::set<int>::iterator it = selectedRaw.begin();

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr src (new pcl::PointCloud<pcl::PointXYZRGB>), target(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::Normal>::Ptr srcN, targetN;
        pcl::PointCloud<pcl::PointNormal>::Ptr srcPN, targetPN;
        pcl::Correspondences corresp;
        Eigen::Matrix4d transform;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr finalPC (new pcl::PointCloud<pcl::PointXYZRGB>), output (new pcl::PointCloud<pcl::PointXYZRGB>);

        double mser = 0;
        double msea = 0;
        int simi = 0;
        if (RW->similarreg.checked)
            simi = RW->similarreg.iter;
        if (RW->absolutereg.checked)
            msea = RW->absolutereg.mse;
        if (RW->relativereg.checked)
            mser = RW->relativereg.mse;

        *src = *DB->getRawPC(*it);
        it++;
        int i = 1;
        for(it; it != selectedRaw.end(); it++)
        {
            LOG("Processing pair number " + std::to_string(i) );
            i++;
            *target = *DB->getRawPC(*it);

            srcN = Core::getNormals(src,RW->paramsreg.radius);
            targetN = Core::getNormals(target,RW->paramsreg.radius);
            srcPN = Core::getNormalPoints(src,RW->paramsreg.radius);
            targetPN = Core::getNormalPoints(target,RW->paramsreg.radius);
            corresp = Core::correspKD(src,target,srcN,targetN);
            int tmp = corresp.size();
            LOG(" ---- Running correspondence rejection on src: " + std::to_string(tmp) + " correspondences.");

            if (RW->mediandistreg.checked)
                corresp = Core::correspRmeddist(src,target,corresp,RW->mediandistreg.medfact);
            if (RW->surfacereg.checked)
                corresp = Core::correspRsurfacenorm(srcPN,targetPN,corresp,RW->surfacereg.angle);
            if (RW->boundaryreg.checked)
                corresp = Core::correspRboudary(srcPN,targetPN,corresp);
            if (RW->one2onereg.checked)
                corresp = Core::correspR121(corresp);
            if (RW->ransacreg.checked)
                corresp = Core::correspRransac(src,target,corresp,RW->ransacreg.iter,RW->ransacreg.threshold);
            LOG(" ------ Total Kept " + std::to_string(corresp.size()) + " correspondences (" + std::to_string( (corresp.size() / (float)tmp) * 100) + "%)");

            LOG(" ------ Running ICP");

            transform = Core::pairRegister(src, target, corresp, RW->methodreg.type, RW->methodreg.maxiter, simi, mser ,msea);
            pcl::transformPointCloud(*src,*output,transform);
            *src = *output;
            *src += *DB->getRawPC(*it);
        }
        DB->addRegisteredPC(src);
        QStringList list;
        list.append("Registered");
        updateRegPCList(list);
    }
}

void MainWindow::updated()
{
    if (selectedRaw.size() > 1)
    {
        LOG("Processing " + std::to_string(selectedRaw.size()) +" point clouds");
        std::set<int>::iterator it = selectedRaw.begin();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr src (new pcl::PointCloud<pcl::PointXYZRGB>), target(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr finalPC (new pcl::PointCloud<pcl::PointXYZRGB>), output (new pcl::PointCloud<pcl::PointXYZRGB>);
        *src = *DB->getRawPC(*it);
        it++;
        int i = 1;
        for(it; it != selectedRaw.end(); it++)
        {
            LOG("Processing pair number " + std::to_string(i) );
            i++;
            *target = *DB->getRawPC(*it);
            LOG(" ------ Running ICP");

            pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
            icp.setInputCloud (src);
            icp.setInputTarget (target);
            icp.setMaxCorrespondenceDistance (0.50);
            icp.setMaximumIterations (100);
            icp.setTransformationEpsilon (1e-10);
            icp.align (*output);
            *src = *output;
            *src += *DB->getRawPC(*it);
        }
        LOG(" ------ ICP finished");
        DB->addRegisteredPC(src);
        QStringList list;
        QString str;
        str.setNum((rand() % 100));
        list.append("Registered" +str);
        updateRegPCList(list);
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
