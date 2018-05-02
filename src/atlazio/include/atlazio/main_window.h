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
#include <QString>

namespace Ui {
class MainWindow;
}

namespace atlazio
{
   class QNode;
   class RosMonitor;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(int argc, char** argv, QWidget *parent = 0);
    ~MainWindow();
    void draw();
private:
    Ui::MainWindow *ui;
    atlazio::QNode* rosNode;
    atlazio::RosMonitor* rosMonitor;
    
public slots:
  void receiveNewPose(const double& x, const double& y);
  void refreshTopics();
  void refreshCustomPlot();
  void onTopicChanged(const QString&);

    
};

#endif // atlazio_MAIN_WINDOW_H
