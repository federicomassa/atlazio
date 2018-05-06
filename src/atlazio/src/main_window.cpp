#include "main_window.h"
#include "ui_main_window.h"
#include "qcustomplot.h"
#include "ros_monitor.h"
#include "ros_node.h"
#include "topic.h"

#include <vector>
#include <string>
#include <QDialogButtonBox>


MainWindow::MainWindow(int argc, char** argv, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    rosNode(new atlazio::RosNode(argc, argv)),
    rosMonitor(new atlazio::RosMonitor(argc, argv)),
    bagReader(new atlazio::BagReader(this)),
    isLiveMode(false)
{
    ui->setupUi(this);
    ui->bagFileLineEdit->setReadOnly(true);
    ui->topicBox->setMinimumSize(QSize(600, 50));
    
    setWindowTitle("ATLAZIO");
    
    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(refreshCustomPlot()));
    timer->start(100);
    
    // Connect refresh button
    connect(ui->refreshTopicsButton, SIGNAL(clicked()), this, SLOT(refreshTopics()));
    connect(ui->closeBagButton, SIGNAL(clicked()), this, SLOT(closeBagFile()));
    
    // topic change signal
    connect(ui->topicBox, SIGNAL(currentIndexChanged(int)), this, SLOT(onTopicChanged(int)));
    
    // Arbitrarily large numbers so that first points in graph define new range
    resetRangeData();
    
    // Graph margin in %
    graphMargin = 0.1;
    
    // Browse files
    connect(ui->browseButton, SIGNAL(clicked()), this, SLOT(openBagFile()));
    
    // Message box connection
    connect(bagReader, SIGNAL(openingBagFile()), this, SLOT(openMessageBox()));
    connect(bagReader, SIGNAL(closingBagFile()), this, SLOT(closeMessageBox()));
    
    connect(ui->liveModeBox, SIGNAL(stateChanged(int)), this, SLOT(liveMode(int)));
}

void MainWindow::resetRangeData()
{
    currentMinX = 1E13;
    currentMaxX = -1E13;
    currentMinY = 1E13;
    currentMaxY = -1E13;
}


MainWindow::~MainWindow()
{
    delete ui;
    delete rosNode;
    delete rosMonitor;
}

void MainWindow::draw()
{
  show();
  //app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
  
//   std::vector<std::string> topics = rosNode->getAvailableTopics(); 
//   qDebug("------- Available topicsc_: -----");
//   for (auto itr = topics.begin(); itr != topics.end(); itr++)
//     qDebug((*itr).c_str());
//   
//   qDebug("-------------------------------");
  

//     rosNode->init();
//     if (!rosNode->isRunning())
//       rosNode->run();
//  
    
    // Refresh button
    ui->refreshTopicsButton->setIcon(QIcon(":/images/refresh.png"));
    ui->closeBagButton->setIcon(QIcon(":/images/close.png"));

    // add two new graphs and set their look:
    ui->signalPlot->addGraph();
    ui->signalPlot->graph(0)->setPen(QPen(Qt::red)); // line color red for first graph
    ui->signalPlot->graph(0)->setLineStyle((QCPGraph::LineStyle)QCPGraph::lsNone);
    ui->signalPlot->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc , 3));
    //     ui->signalPlot->graph(0)->setBrush(QBrush(QColor(0, 0, 255, 20))); // first graph will be filled with translucent blue
    
    // configure right and top axis to show ticks but no labels:
    // (see QCPAxisRect::setupFullAxesBox for a quicker method to do this)
    ui->signalPlot->xAxis->setRange(-10, 10);
    ui->signalPlot->yAxis->setRange(-10,10);
    
    ui->signalPlot->xAxis2->setVisible(true);
    ui->signalPlot->xAxis2->setTickLabels(false);
    ui->signalPlot->yAxis2->setVisible(true);
    ui->signalPlot->yAxis2->setTickLabels(false);
    // make left and bottom axes always transfer their ranges to right and top axes:
    //connect(ui->signalPlot->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->signalPlot->xAxis2, SLOT(setRange(QCPRange)));
    //connect(ui->signalPlot->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->signalPlot->yAxis2, SLOT(setRange(QCPRange)));
    // pass data points to graphs:
    // let the ranges scale themselves so graph 0 fits perfectly in the visible area:
    ui->signalPlot->graph(0)->rescaleAxes();
    // same thing for graph 1, but only enlarge ranges (in case graph 1 is smaller than graph 0):
    // Note: we could have also just called ui->signalPlot->rescaleAxes(); instead
    // Allow user to drag axis ranges with mouse, zoom with mouse wheel and select graphs by clicking:
    ui->signalPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    
    
    
    // add two new graphs and set their look:
    ui->trackPlot->addGraph();
    ui->trackPlot->graph(0)->setPen(QPen(Qt::blue)); // line color red for first graph
    ui->trackPlot->graph(0)->setLineStyle((QCPGraph::LineStyle)QCPGraph::lsNone);
    ui->trackPlot->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc , 3));
    //     ui->trackPlot->graph(0)->setBrush(QBrush(QColor(0, 0, 255, 20))); // first graph will be filled with translucent blue
    
    // configure right and top axis to show ticks but no labels:
    // (see QCPAxisRect::setupFullAxesBox for a quicker method to do this)
    ui->trackPlot->xAxis->setRange(-10, 10);
    ui->trackPlot->yAxis->setRange(-10,10);
    
    ui->trackPlot->xAxis2->setVisible(true);
    ui->trackPlot->xAxis2->setTickLabels(false);
    ui->trackPlot->yAxis2->setVisible(true);
    ui->trackPlot->yAxis2->setTickLabels(false);
    // make left and bottom axes always transfer their ranges to right and top axes:
    //connect(ui->trackPlot->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->trackPlot->xAxis2, SLOT(setRange(QCPRange)));
    //connect(ui->trackPlot->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->trackPlot->yAxis2, SLOT(setRange(QCPRange)));
    // pass data points to graphs:
    // let the ranges scale themselves so graph 0 fits perfectly in the visible area:
    ui->trackPlot->graph(0)->rescaleAxes();
    // same thing for graph 1, but only enlarge ranges (in case graph 1 is smaller than graph 0):
    // Note: we could have also just called ui->trackPlot->rescaleAxes(); instead
    // Allow user to drag axis ranges with mouse, zoom with mouse wheel and select graphs by clicking:
    //ui->trackPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    
}

void MainWindow::receiveNewPose(const double& x, const double& y)
{
  qDebug() << "received new pose: " << x << '\t' << y;
  if (ui->signalPlot->graph(0) == nullptr)
    return;
  
  if (x > currentMaxX)
    currentMaxX = x;
  if (x < currentMinX)
    currentMinX = x;
  
  if (y > currentMaxY)
    currentMaxY = y;
  if (y < currentMinY)
    currentMinY = y;
  
  ui->signalPlot->graph(0)->addData(x, y);
  qDebug() << "added new point";
}

void MainWindow::refreshCustomPlot()
{
  ui->signalPlot->xAxis->setRange(currentMinX - graphMargin*(currentMaxX - currentMinX), currentMaxX + graphMargin*(currentMaxX - currentMinX));
  ui->signalPlot->yAxis->setRange(currentMinY - graphMargin*(currentMaxY - currentMinY), currentMaxY + graphMargin*(currentMaxY - currentMinY));//   ui->signalPlot->replot(); 

  ui->signalPlot->replot(); 
}


void MainWindow::refreshTopics()
{
   ui->topicBox->clear();
  
   if (isLiveMode)
   {
    if (rosMonitor->isRunning())
      rosMonitor->wait();
   
    rosMonitor->init();

    qDebug("Run started from main window");
    
    rosMonitor->wait();
    
    const QMap<QString, QString>& availableTopics = rosMonitor->getAvailableTopics();
    for (auto itr = availableTopics.begin(); itr != availableTopics.end(); itr++)
    {
      ui->topicBox->addTopic(atlazio::Topic(itr.key(), itr.value()));
    }
   }
   
   else
   {
     if (bagReader->isOpen())
     {
	QMap<QString,QString> availableTopics = bagReader->getAvailableTopics();
	
	for (auto itr = availableTopics.begin(); itr != availableTopics.end(); itr++)
	{
	  ui->topicBox->addTopic(atlazio::Topic(itr.key(), itr.value()));
	}
     }
   }
}


void MainWindow::onTopicChanged(int index)
{
 // rosNode->terminate();
  atlazio::Topic t = ui->topicBox->getTopic(index);
  
  if (isLiveMode)
  {
    if (ui->signalPlot->graph(0))
    {
      ui->signalPlot->graph(0)->data()->clear();
      ui->signalPlot->replot();
    }
  
    resetRangeData();
  
    if (rosNode->isRunning())
    {
      rosNode->quit();
      rosNode->wait();
    }
    
    qDebug() << "onTopicChanged: " << t.getName() << " " << t.getType();
    rosNode->init(t.getName(), t.getType());
//   rosNode->run();
  //emit changedTopic(newTopic, availableTopics.value(newTopic));
  }
  
  else
  {
    
  }
    
}

void MainWindow::openBagFile()
{
   QString filename =  QFileDialog::getOpenFileName(
          this,
          "Open bag file",
          QDir::currentPath(),
          "All files (*.*) ;; Bag files (*.bag)");
 
    if( !filename.isNull() )
    {
     bagReader->open(filename.toLatin1().data());
     bagReader->drawTrack(ui->trackPlot, "/odom");
    }
    
    ui->bagFileLineEdit->setText(filename);
    refreshTopics();
    //bagReader->close();
}

void MainWindow::openMessageBox()
{
    //waitMsg = new QDialog(QMessageBox::Icon::Information, QString(""), QString("Loading..."), QMessageBox::NoButton, this);
  waitMsg = new QProgressDialog(this);
  waitMsg->setModal(false);
  waitMsg->setMinimumDuration(0);
  waitMsg->setValue(1);
  waitMsg->show();
  waitMsg->raise();
  waitMsg->activateWindow();
}


void MainWindow::closeMessageBox()
{
  waitMsg->close();
  delete waitMsg;
  waitMsg = nullptr;
}

void MainWindow::terminateThreads()
{
  rosMonitor->quit();
  rosMonitor->wait();
  
  rosNode->quit();
  rosNode->wait();
}

void MainWindow::liveMode(int status)
{
   if (status > 0)
   { 
     isLiveMode = true;
     
     connect(this, SIGNAL(changedTopic(const QString&, const QString&)), rosNode, 
	    SLOT(changeSubscription(const QString&, const QString&)));
     connect(rosNode, SIGNAL(newXYPoint(const double&, const double&)), this, SLOT(receiveNewPose(const double&, const double&)));
    
   
     // Close bag file
     
    
     // Live 
        rosMonitor->init();
//     if (!rosMonitor->isRunning())
//       rosMonitor->run();
    
	rosNode->init("", "");
//     if (!rosNode->isRunning())
//       rosNode->run();

    
      rosMonitor->wait();
      
      refreshTopics();
   }
   else if (status == 0)
   {
     // Disconnect everything and 
     disconnect(this, SIGNAL(changedTopic(const QString&, const QString&)), rosNode, 
	    SLOT(changeSubscription(const QString&, const QString&)));
     disconnect(rosNode, SIGNAL(newXYPoint(const double&, const double&)), this, SLOT(receiveNewPose(const double&, const double&)));
    
     
     isLiveMode = false;
     
     if (rosMonitor->isRunning())
     {
       rosMonitor->quit();
       rosMonitor->wait();
     }
     
     if (rosNode->isRunning())
     {
      rosNode->quit();
      rosNode->wait();
     }
     
     refreshTopics();
   }
}

void MainWindow::closeBagFile()
{
  bagReader->close();
  refreshTopics();
  
  ui->bagFileLineEdit->setText("");
  
  if (ui->trackPlot->graph(0))
    ui->trackPlot->graph(0)->data()->clear();
  
  ui->trackPlot->replot();
}




