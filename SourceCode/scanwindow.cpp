#include "scanwindow.h"
#include "Storage/database.h"
#include "ui_scanwindow.h"
#include <string>
#include <IO/kinect_v2.h>
#include <IO/kinect2_grabber.h>
#include <logger.h>

using namespace std;

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

/// @author: Mladen Rakic
/// @date: 26-12-2017
/// @version 1.0
///
/// @brief Function to display text from logger file in scan window.
/// @param filename Name of the text file to be read
void scanwindow::readfile(std::string filename){
//    QFile file(filename);
//    if(!file.exists()){
//        qDebug() << "File not found! "<<filename;
//    }else{
//        qDebug() << filename<<" File found";
//    }
//    std::string line;
//    ui->sw_logger_textedit->clear();
//    if (file.open(QIODevice::ReadOnly | QIODevice::Text)){
//        QTextStream stream(&file);
//        while (!stream.atEnd()){
//            line = stream.readLine();
//            ui->sw_logger_textedit->setText(ui->sw_logger_textedit->toPlainText()+line+"\n");
//            qDebug() << "Line: "<<line;
//        }
//    }
//    file.close();
}

void scanwindow::on_sw_horizontalacq_radiobutton_clicked()
{

}

void scanwindow::on_sw_startscan_pushbutton_clicked()
{
    pcl::Kinect2Grabber * kinect = new pcl::Kinect2Grabber;
    kinect->start();

    // Credits: UnaNancyOwen (https://github.com/UnaNancyOwen/KinectGrabber/blob/Kinect2Grabber/Sample/main.cpp)
    // PCL Visualizer
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
            new pcl::visualization::PCLVisualizer( "Point Cloud Viewer" ) );
        viewer->setCameraPosition( 0.0, 0.0, -2.5, 0.0, 0.0, 0.0 );


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

        // Kinect2Grabber
//        boost::shared_ptr<pcl::Grabber> grabber = boost::make_shared<pcl::Kinect2Grabber>();

        // Register Callback Function
        boost::signals2::connection connection = grabber->registerCallback( function );

        // Start Grabber
        grabber->start();

        while( !viewer->wasStopped() ){
            // Update Viewer
            viewer->spinOnce();

            boost::mutex::scoped_try_lock lock( mutex );
            if( lock.owns_lock() && cloud ){
                // Update Point Cloud
                if( !viewer->updatePointCloud( cloud, "cloud" ) ){
                    viewer->addPointCloud( cloud, "cloud" );
                }
            }
        }
}

void scanwindow::on_sw_stopscan_pushbutton_clicked()
{


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

    // Register Callback Function
    boost::signals2::connection connection = grabber->registerCallback( function );

    // Stop Grabber
    grabber->stop();

    // Disconnect Callback Function
    if( connection.connected() ){
        connection.disconnect();
    }
}
