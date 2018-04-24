/**
 * @file /include/atlazio/main_window.hpp
 *
 * @brief Qt based gui for atlazio.
 *
 * @date November 2010
 **/
#ifndef atlazio_MAIN_WINDOW_H
#define atlazio_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/
#include <QMainWindow>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void draw();
private:
    Ui::MainWindow *ui;
    
};

#endif // atlazio_MAIN_WINDOW_H
