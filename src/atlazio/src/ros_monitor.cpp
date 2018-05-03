/**
 * @file /src/ros_monitor.cpp
 *
 * @brief A node to retrieve information on ros master
 *
 * @date May 2018
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "ros_monitor.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace atlazio {

/*****************************************************************************
** Implementation
*****************************************************************************/

RosMonitor::RosMonitor(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

RosMonitor::~RosMonitor() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool RosMonitor::init() {
	ros::init(init_argc,init_argv,"atlazio_ros_monitor");
	
	if ( ! ros::master::check() ) 
	{
	  qDebug("RosMonitor::init - Unable to contact master");
	  return false;
	}
	
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	start();
		
	return true;
}

void RosMonitor::run() {
  qDebug("RosMonitor::run");
  ros::master::V_TopicInfo topics;
  ros::master::getTopics(topics);
  
  availableTopics.clear();
  
  for (auto itr = topics.begin(); itr != topics.end(); itr++)
    availableTopics[QString(itr->name.c_str())] = QString(itr->datatype.c_str());
  
  qDebug("End of RosMonitor::run");
}




const QMap< QString, QString >& RosMonitor::getAvailableTopics() const
{
  return availableTopics;
}


}  // namespace atlazio


