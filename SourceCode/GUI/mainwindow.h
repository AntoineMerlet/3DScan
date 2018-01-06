#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QVTKWidget.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>
#include "Storage/database.h"
#include <QStandardItemModel>
#include <GUI/filterwindow.h>
#include <GUI/regwindow.h>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);

    ~MainWindow();

    void MainWindow::readfile(std::string);

protected:

    void updatePCList(QStringList);
    void updateRegPCList(QStringList);
    void updateMeshList(QStringList);

private slots:
    void on_actionNew_scan_triggered();
    void on_actionImport_point_clouds_triggered();
    void on_actionImport_registered_PC_triggered();
    void on_actionImport_mesh_triggered();
    void on_actionExport_point_clouds_triggered();
    void on_actionExport_registered_PC_triggered();
    void on_actionExport_mesh_triggered();
    void on_mw_register_pc_pushbutton_clicked();
    void on_mw_generatemesh_pushbutton_clicked();
    void on_actionAbout_triggered();
    void on_actionUser_manual_triggered();

    void on_filter_pb_clicked();

    void on_pc_list_clicked(const QModelIndex &index);

    void on_regpc_list_clicked(const QModelIndex &index);

    void on_mesh_list_clicked(const QModelIndex &index);

public slots:
    void receivedmessage(const QString& arg);

private:
    Ui::MainWindow *ui;
    DataBase *DB;
    void UpdateSelectedRaw();
    void UpdateSelectedReg();
    void UpdateSelectedMesh();
    void updateDisplay();
    std::list<int> selectedRaw;
    std::list<int> selectedRegistered;
    std::list<int> selectedMeshed;
    QStandardItemModel * PCList;
    QStandardItemModel * RPCList;
    QStandardItemModel * MeshList;
    QVTKWidget pcScene;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> pcViz;
};

#endif // MAINWINDOW_H
