/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QApplication>
#include "main_window.h"

#include <ros/ros.h>

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

    /*********************
    ** Qt
    **********************/
    ros::init(argc,argv,"atlazio_ros_node");

    QApplication app(argc, argv);
    MainWindow w(argc, argv);
    w.draw();
    
    return app.exec();
}
