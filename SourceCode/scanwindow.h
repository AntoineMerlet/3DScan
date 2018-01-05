#ifndef SCANWINDOW_H
#define SCANWINDOW_H

#include <QMainWindow>
#include <QVTKWidget.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>
#include <IO/kinect_v2.h>
#include <IO/kinect2_grabber.h>
#include <vtkBoundingBox.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

namespace Ui {
class scanwindow;
}

class scanwindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit scanwindow(QWidget *parent = 0);
    static int counter;
    ~scanwindow();

private slots:
    void on_sw_horizontalacq_radiobutton_clicked();

    void on_sw_horizontalacq_radiobutton_clicked(bool checked);

    void on_sw_verticalacq_radiobutton_clicked(bool checked);

    void on_sw_xmin_horslider_actionTriggered(int action);

    void on_sw_xmax_horslider_actionTriggered(int action);

    void on_sw_ymin_horslider_actionTriggered(int action);

    void on_sw_ymax_horslider_actionTriggered(int action);

    void on_sw_zmin_horslider_actionTriggered(int action);

    void on_sw_zmax_horslider_actionTriggered(int action);

    void on_sw_startscan_pushbutton_clicked();

private:
    Ui::scanwindow *ui;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    boost::shared_ptr<pcl::Grabber> grabber = boost::make_shared<pcl::Kinect2Grabber>();


};

#endif // SCANWINDOW_H
