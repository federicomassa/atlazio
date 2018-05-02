#include "main_window.h"
#include "ui_main_window.h"
#include "qcustomplot.h"
#include "qnode.h"
#include "ros_monitor.h"

#include <vector>
#include <string>

MainWindow::MainWindow(int argc, char** argv, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    rosNode(new atlazio::QNode(argc, argv)),
    rosMonitor(new atlazio::RosMonitor(argc, argv))
{
    ui->setupUi(this);
    
    connect(rosNode, SIGNAL(poseReceived(const double&, const double&)), 
			    this, SLOT(receiveNewPose(const double&, const double&)));
    
    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(refreshCustomPlot()));
    timer->start(100);
    
  
    
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
//   qDebug("------- Available topics: -----");
//   for (auto itr = topics.begin(); itr != topics.end(); itr++)
//     qDebug((*itr).c_str());
//   
//   qDebug("-------------------------------");
  

    rosNode->init();
    if (!rosNode->isRunning())
      rosNode->run();
    
    rosMonitor->init();
    if (!rosMonitor->isRunning())
      rosMonitor->run();

    qDebug("Run started from main window");
    
    rosMonitor->wait();
    
    std::vector<std::string> availableTopics = rosMonitor->getAvailableTopics();
    for (auto itr = availableTopics.begin(); itr != availableTopics.end(); itr++)
    {
      ui->comboBox->addItem(QString(itr->c_str()));
    }
    
    
    // add two new graphs and set their look:
    ui->customPlot->addGraph();
    ui->customPlot->graph(0)->setPen(QPen(Qt::red)); // line color red for first graph
    ui->customPlot->graph(0)->setLineStyle((QCPGraph::LineStyle)QCPGraph::lsNone);
    ui->customPlot->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc , 3));
    //     ui->customPlot->graph(0)->setBrush(QBrush(QColor(0, 0, 255, 20))); // first graph will be filled with translucent blue
    
    // configure right and top axis to show ticks but no labels:
    // (see QCPAxisRect::setupFullAxesBox for a quicker method to do this)
    ui->customPlot->xAxis2->setVisible(true);
    ui->customPlot->xAxis2->setTickLabels(false);
    ui->customPlot->yAxis2->setVisible(true);
    ui->customPlot->yAxis2->setTickLabels(false);
    // make left and bottom axes always transfer their ranges to right and top axes:
    //connect(ui->customPlot->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->customPlot->xAxis2, SLOT(setRange(QCPRange)));
    //connect(ui->customPlot->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->customPlot->yAxis2, SLOT(setRange(QCPRange)));
    // pass data points to graphs:
    // let the ranges scale themselves so graph 0 fits perfectly in the visible area:
    ui->customPlot->graph(0)->rescaleAxes();
    // same thing for graph 1, but only enlarge ranges (in case graph 1 is smaller than graph 0):
    // Note: we could have also just called ui->customPlot->rescaleAxes(); instead
    // Allow user to drag axis ranges with mouse, zoom with mouse wheel and select graphs by clicking:
    ui->customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
}

void MainWindow::receiveNewPose(const double& x, const double& y)
{
  ui->customPlot->graph(0)->addData(x, y);
}

void MainWindow::refreshCustomPlot()
{
   ui->customPlot->replot();
}