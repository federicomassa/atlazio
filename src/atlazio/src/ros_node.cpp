/**
 * @file /src/ros_node.cpp
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
#include "ros_node.h"
#include <QDebug>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace atlazio {

/*****************************************************************************
** Implementation
***************
**************************************************************/

void RosNode::odometryCallback(const nav_msgs::Odometry_< std::allocator< void > >::ConstPtr& msg)
{
  //qDebug("odometryCallback: %f, %f", msg->pose.pose.position.x, msg->pose.pose.position.y);
  emit newXYPoint(msg->pose.pose.position.x, msg->pose.pose.position.y);
}

void RosNode::imuCallback(const sensor_msgs::Imu_< std::allocator< void > >::ConstPtr& msg)
{
  //qDebug("imuCallback: %f, %f", msg->linear_acceleration.x, msg->linear_acceleration.y);
  emit newXYPoint(msg->linear_acceleration.x, msg->linear_acceleration.y);
}


RosNode::RosNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv),
	spinner(1)
	{
	  
	}

RosNode::~RosNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool RosNode::init(const QString& topicName, const QString& topicType) {
	currentTopicName = topicName;
	currentTopicType = topicType;
	
	
  if ( ! ros::master::check() ) 
  {
	  qDebug("RosNode::init - Unable to contact master");
	  return false;
  }
	
  sub.shutdown();
    
  if (currentTopicType == "nav_msgs/Odometry")
    sub = n.subscribe(currentTopicName.toLatin1().data(), 1000, &RosNode::odometryCallback, this);
  else if (currentTopicType == "sensor_msgs/Imu")
    sub = n.subscribe(currentTopicName.toLatin1().data(), 1000, &RosNode::imuCallback, this);
	
  
  qDebug() << "RosNode::init" << topicName << topicType;
  //ros::start(); // explicitly needed since our nodehandle is going out of scope.
  start();
		
  return true;
}

void RosNode::run() {
  qDebug("RosNode::run");
  
  spinner.start();	
  //ros::waitForShutdown();
  
  exec();
}

const std::vector< std::string >& RosNode::getAvailableTopics() const
{
  return std::vector<std::string>();
}

void RosNode::changeSubscription(const QString& newTopicName, const QString& newTopicType)
{ 
  currentTopicName = newTopicName;
  currentTopicType = newTopicType;
}

void RosNode::slot2()
{
   qDebug("slot2!");
}


}  // namespace atlazio


