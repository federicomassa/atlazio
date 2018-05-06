#include "bag_reader.h"
#include "qcustomplot.h"

#include <nav_msgs/Odometry.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

namespace atlazio 
{

BagReader::BagReader(QWidget* parent) : QObject(parent)
{
  bagPath = "";
  
  trackMinX = 1E13;
  trackMaxX = -1E13;
  trackMinY = 1E13;
  trackMaxY = -1E13; 
  
  graphMargin = 0.1;
}
  
bool BagReader::isOpen() const
{
  return (bagPath != "");
}

void BagReader::close()
{
  bag.close();
  bagPath = "";
}

void BagReader::open(const std::string& fileName)
{
  bagPath = fileName;
  bag.open(fileName.c_str());
}

void BagReader::drawTrack(QCustomPlot* plot, const std::string& gpsTopic)
{
  if (plot == nullptr)
  {
      qDebug("BagReader::drawTrack --- ERROR, cannot pass empty plot");
      return;
  }
  
  if (plot->graph(0) == nullptr)
  {
      qDebug("BagReader::drawTrack --- ERROR, plot should have a graph");
      return;
  }
  
  
  std::vector<std::string> topics;
  topics.push_back(gpsTopic);
  
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  
  // Old and new messages
  nav_msgs::Odometry::ConstPtr odom_msg;
  ros::Time initTime;
  
  bool first = true;
  
  nav_msgs::Odometry odom_last_msg;
  
  emit openingBagFile();
  
  
  BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {    
      odom_msg = m.instantiate<nav_msgs::Odometry>();
      
      if (odom_msg != NULL)
      {
	if (first)
	{
	  initTime = odom_msg->header.stamp; 
	  first = false;
	}
	
	double x = odom_msg->pose.pose.position.x;
	double y = odom_msg->pose.pose.position.y;
	
	if (x > trackMaxX)
	  trackMaxX = x;
	if (x < trackMinX)
	  trackMinX = x;
	
	if (y > trackMaxY)
	  trackMaxY = y;
	if (y < trackMinY)
	  trackMinY = y;
	
	plot->graph(0)->addData(x, y);
	
      }
      

    }
    
    plot->xAxis->setRange(trackMinX - graphMargin*(trackMaxX - trackMinX), trackMaxX + graphMargin*(trackMaxX - trackMinX));
    plot->yAxis->setRange(trackMinY - graphMargin*(trackMaxY - trackMinY), trackMaxY + graphMargin*(trackMaxY - trackMinY));
    plot->replot();
}

QMap<QString, QString> BagReader::getAvailableTopics() const
{
  rosbag::View view(bag);
  std::vector<const rosbag::ConnectionInfo *> connection_infos = view.getConnections();
  QMap<QString, QString> topics;
  
  BOOST_FOREACH(const rosbag::ConnectionInfo *info, connection_infos)
  {
    topics[QString(info->topic.c_str())] = QString(info->datatype.c_str());    
  }

  return topics;
}

  
}