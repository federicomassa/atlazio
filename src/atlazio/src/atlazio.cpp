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
    
    
    app.setWindowIcon(QIcon(":/images/logo.jpeg"));
    MainWindow w(argc, argv);
    
    app.connect(&app, SIGNAL(lastWindowClosed()), &w, SLOT(terminateThreads())); 
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit())); 
    
    
    w.draw();
    
    return app.exec();
}
