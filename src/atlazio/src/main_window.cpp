/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/atlazio/main_window.hpp"
#include <qcustomplot.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace atlazio {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    
	setWindowIcon(QIcon(":/images/icon.png"));
}

MainWindow::~MainWindow() {}

}