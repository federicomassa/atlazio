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
#include <QMap>
#include "bag_reader.h"
#include <QProgressDialog>

namespace Ui {
class MainWindow;
}

namespace atlazio
{
   class RosMonitor;
   class RosNode;
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
    atlazio::RosNode* rosNode;
    atlazio::RosMonitor* rosMonitor;
    
    // Utility class to open bag files
    atlazio::BagReader* bagReader;
    
    // Range of current graph
    double currentMinX;
    double currentMaxX;
    double currentMinY;
    double currentMaxY;
    
    // Message box
    QProgressDialog* waitMsg;
    
    // Additional space left in graph beyond data range (in %)
    double graphMargin;
    
    void resetRangeData();
    bool isLiveMode;
public slots:
  void receiveNewPose(const double& x, const double& y);
  void refreshTopics();
  void refreshCustomPlot();
  void onTopicChanged(int);
  void openBagFile();
  void openMessageBox();
  void closeMessageBox();
  void terminateThreads();
  void liveMode(int);
  void closeBagFile();
    
signals:
  void changedTopic(const QString& newTopicName, const QString& newTopicType);
  
};

#endif // atlazio_MAIN_WINDOW_H
