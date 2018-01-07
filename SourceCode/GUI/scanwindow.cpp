#include "scanwindow.h"
#include "Storage/database.h"
#include "ui_scanwindow.h"
#include <string>
#include <GUI/mainwindow.h>
#include <IO/kinect_v2.h>
#include <IO/kinect2_grabber.h>
#include <IO/logger.h>
#include <QSlider>
#include <QCloseEvent>
#include <QObject>

using namespace std;

int scanwindow::counter = 0;

scanwindow::scanwindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::scanwindow)
{
    ui->setupUi(this);
}

scanwindow::~scanwindow()
{
    QApplication::exit();
    viewer->close();
    delete ui;
}

/// @author: Mladen Rakic
/// @date: 24-12-2017
/// @version 1.0
///
/// @brief Function used to choose vertical acquisition mode.
/// @param bool indicator of the button pressed
void scanwindow::on_sw_verticalacq_radiobutton_clicked(bool checked)
{
    ui->sw_horizontalacq_radiobutton->setChecked(false);
}

/// @author: Mladen Rakic
/// @date: 24-12-2017
/// @version 1.0
///
/// @brief Function used to choose horizontal acquisition mode.
/// @param bool indicator of the button pressed
void scanwindow::on_sw_horizontalacq_radiobutton_clicked(bool checked)
{
    ui->sw_verticalacq_radiobutton->setChecked(false);
}

/// @author: Mladen Rakic / Marcio Rockenbach
/// @date: 04-01-2018
/// @version 1.0
///
/// @brief Function used to start the live scan
void scanwindow::on_sw_startscan_pushbutton_clicked()
{
    kinect = new pcl::Kinect2Grabber;
    kinect->start();
    scan_pressed = true;
    // Credits: UnaNancyOwen (https://github.com/UnaNancyOwen/KinectGrabber/blob/Kinect2Grabber/Sample/main.cpp)
    // PCL Visualizer
    viewer.reset(new pcl::visualization::PCLVisualizer("Kinect viewer"));
    viewer->setCameraPosition( 0.0, 0.0, -2.5, 0.0, 0.0, 0.0 );
    viewer->setShowFPS(false);
    viewer->addText("Press 's' on keyboard to save",0,0);
    updatebox();

    // Point Cloud
        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud;

        // Retrieved Point Cloud Callback Function
        boost::mutex mutex;
        boost::function<void( const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& )> function =
            [&cloud, &mutex]( const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& ptr ){
                boost::mutex::scoped_lock lock( mutex );

                /* Point Cloud Processing */
                cloud = ptr->makeShared();
            };

        // Keyboard Callback Function
        boost::function<void( const pcl::visualization::KeyboardEvent& )> keyboard =
            [&cloud, &mutex]( const pcl::visualization::KeyboardEvent& event ){
                if( event.getKeySym() == "s" && event.keyDown() ){
                    boost::mutex::scoped_lock lock( mutex );
                    // Generate Indices ( 000, 001, 002, ... )
                    static uint32_t index = 0;
                    std::ostringstream oss;
                    oss << std::setfill( '0' ) << std::setw( 3 ) << index++;
                    std::cout << oss.str() + ".pcd" << std::endl;
                    // Save Point Cloud to PLY File
                    scanwindow::counter = scanwindow::counter + 1;
                    pcl::io::savePLYFile( oss.str() + ".ply", *cloud );
                }
            };

        // Register Keyboard Callback Function
        viewer->registerKeyboardCallback( keyboard );

        // Register Callback Function
        boost::signals2::connection connection = grabber->registerCallback( function );

        // Start Grabber
        grabber->start();

        while( !viewer->wasStopped() ){
                // Update Viewer
                viewer->spinOnce();
                ui->cap_pc->setValue(counter);
                boost::mutex::scoped_try_lock lock( mutex );
                if( lock.owns_lock() && cloud ){
                    // Update Point Cloud
                    if( !viewer->updatePointCloud( cloud, "cloud" ) ){
                        viewer->addPointCloud( cloud, "cloud" );
                    }
                }
            }

            // Stop Grabber
            grabber->stop();

            // Disconnect Callback Function
            if( connection.connected() ){
                connection.disconnect();
            }
}

/// @author: Mladen Rakic / Marcio Rockenbach
/// @date: 05-01-2018
/// @version 1.0
///
/// @brief Function used to update the bounding box on the live feed from the sensor
void scanwindow::updatebox(){
    viewer->removeShape("cube");
    float x_min = ((float)ui->xmin->value())/3;
    float x_max = ((float)ui->xmax->value())/3;
    float y_min = ((float)ui->ymin->value())/3;
    float y_max = ((float)ui->ymax->value())/3;
    float z_min = ((float)ui->zmin->value())/3;
    float z_max = ((float)ui->zmax->value())/3;
    viewer->addCube(x_min, x_max, y_min, y_max, z_min, z_max);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "cube");
    viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.8, 0.0, 0.0, "cube");
    viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "cube");
    viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.33, "cube");

}

void scanwindow::on_xmin_sliderReleased()
{
    ui->xmin->setValue(ui->xmin->value());
}

void scanwindow::on_sw_stopscan_pushbutton_clicked()
{
    if (scan_pressed == true){
            viewer->close();
            grabber->stop();
            kinect->stop();
            scan_pressed = false;}
    emit send_unhide();
    scanwindow::setVisible(false);
}

void scanwindow::on_xmin_sliderMoved(int position)
{
    updatebox();
}

void scanwindow::on_xmax_sliderMoved(int position)
{
    updatebox();
}

void scanwindow::on_ymin_sliderMoved(int position)
{
    updatebox();
}

void scanwindow::on_ymax_sliderMoved(int position)
{
    updatebox();
}

void scanwindow::on_zmin_sliderMoved(int position)
{
    updatebox();
}

void scanwindow::on_zmax_sliderMoved(int position)
{
    updatebox();
}

void scanwindow::closeEvent(QCloseEvent *e){
    emit send_unhide();
    if (scan_pressed == true)
        viewer->close();
    e->accept();
}
