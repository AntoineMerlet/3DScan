#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QVTKWidget.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>
#include "Storage/database.h"

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

    void resetVtk();

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


private:
    Ui::MainWindow *ui;
    DataBase *DB;

    QVTKWidget pcScene;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> pcViz;
};

#endif // MAINWINDOW_H
