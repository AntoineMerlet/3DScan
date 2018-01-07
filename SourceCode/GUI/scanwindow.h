#ifndef SCANWINDOW_H
#define SCANWINDOW_H

#include <QMainWindow>
#include <pcl/visualization/pcl_visualizer.h>
#include <iomanip>
#include <pcl/io/pcd_io.h>
#include <IO/kinect2_grabber.h>


namespace Ui {
class scanwindow;
}

class scanwindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit scanwindow(QWidget *parent = 0);
    static int counter;
    void updatebox();
    ~scanwindow();

private slots:
    void on_sw_horizontalacq_radiobutton_clicked();
    void on_sw_horizontalacq_radiobutton_clicked(bool checked);
    void on_sw_verticalacq_radiobutton_clicked(bool checked);
    void on_sw_startscan_pushbutton_clicked();
    void on_sw_stopscan_pushbutton_clicked();
    void on_xmin_sliderMoved(int);
    void on_xmax_sliderMoved(int);
    void on_ymin_sliderMoved(int);
    void on_ymax_sliderMoved(int);
    void on_zmin_sliderMoved(int);
    void on_zmax_sliderMoved(int);

signals:
    void send_unhide();

private:
    bool scan_pressed = false;
    Ui::scanwindow *ui;
    pcl::Kinect2Grabber * kinect;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    boost::shared_ptr<pcl::Grabber> grabber = boost::make_shared<pcl::Kinect2Grabber>();
    void closeEvent(QCloseEvent*);
};

#endif // SCANWINDOW_H
