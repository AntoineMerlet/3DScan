#include "scanwindow.h"
#include "Storage/database.h"
#include "ui_scanwindow.h"
#include <string>
#include <IO/kinect_v2.h>
#include <IO/kinect2_grabber.h>
#include <logger.h>

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
    //...
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
    //...
}

//Modify the spatial coordinates
void scanwindow::on_sw_xmin_horslider_actionTriggered(int action)
{
   //dataclass->set_xmin(action);
}

void scanwindow::on_sw_xmax_horslider_actionTriggered(int action)
{
    //dataclass->set_xmax(action);
}

void scanwindow::on_sw_ymin_horslider_actionTriggered(int action)
{
    //dataclass->set_ymin(action);
}

void scanwindow::on_sw_ymax_horslider_actionTriggered(int action)
{
    //dataclass->set_ymax(action);
}

void scanwindow::on_sw_zmin_horslider_actionTriggered(int action)
{
    //dataclass->set_zmin(action);
}

void scanwindow::on_sw_zmax_horslider_actionTriggered(int action)
{
    //dataclass->set_zmax(action);
}

void scanwindow::on_sw_horizontalacq_radiobutton_clicked()
{

}

/// @author: Mladen Rakic / Marcio Rockenbach
/// @date: 04-01-2018
/// @version 1.0
///
/// @brief Function used to start the live scan
void scanwindow::on_sw_startscan_pushbutton_clicked()
{
    vtkBoundingBox box;
    pcl::Kinect2Grabber * kinect = new pcl::Kinect2Grabber;
    kinect->start();

    // Credits: UnaNancyOwen (https://github.com/UnaNancyOwen/KinectGrabber/blob/Kinect2Grabber/Sample/main.cpp)
    // PCL Visualizer
    viewer.reset(new pcl::visualization::PCLVisualizer("Kinect viewer"));
    viewer->setCameraPosition( 0.0, 0.0, -2.5, 0.0, 0.0, 0.0 );

    //        LOG("Background");
    //        ui->kinect_live->SetRenderWindow(viewer->getRenderWindow());
    //        LOG("Render");
    //        viewer->setupInteractor(ui->kinect_live->GetInteractor(),ui->kinect_live->GetRenderWindow());
    //        LOG("Setup");
    //        ui->kinect_live->update();


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
                    // Save Point Cloud to PCD File
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
