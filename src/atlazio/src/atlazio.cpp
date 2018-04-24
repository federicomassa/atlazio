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
    ros::init(argc, argv, "test_node");
    ros::NodeHandle n("~");
    
    ros::Rate rate(1);
    
    QApplication app(argc, argv);
    MainWindow w;
    w.draw();
    
    return app.exec();
}
