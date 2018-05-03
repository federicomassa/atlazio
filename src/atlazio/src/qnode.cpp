/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "qnode.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace atlazio {

/*****************************************************************************
** Implementation
*****************************************************************************/

void QNode::poseCallback(const nav_msgs::Odometry::ConstPtr& odom)
{	
  emit poseReceived(odom->pose.pose.position.x, odom->pose.pose.position.y);
}

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"atlazio_rosNode");
	
	if ( ! ros::master::check() ) 
	{
	  qDebug("QNode::init - Unable to contact master");
	  return false;
	}
	
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	pose_subscriber = n.subscribe("odom_raw", 1000, &QNode::poseCallback, this);
	start();
		
	return true;
}

void QNode::run() {
	//ros::spin();
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

std::vector< std::string > QNode::getAvailableTopics()
{/*
  qDebug("hey");
  ros::master::V_TopicInfo topics;
  ros::master::getTopics(topics);
  
  std::vector<std::string> topicNames;
  for (auto itr = topics.begin(); itr != topics.end(); itr++)
    topicNames.push_back(itr->name);
  
  qDebug("nice");
  
  return topicNames;
  */
}

// void QNode::changeSubscription(const QString& newTopicName, const QString& newTopicType)
// {
//     ros::NodeHandle n;
//     
//     if (newTopicType == "nav_msgs/Odometry")
//       n.subscribe(newTopicName.toLatin1().data(), 1000, &RosNode::odometryCallback, this);
//     else if (newTopicType == "sensor_msgs/Imu")
//       n.subscribe(newTopicName.toLatin1().data(), 1000, &RosNode::imuCallback, this);
//     
//     qDebug() << "New type: " << newTopicType;
//     //ros::spin();
// }


}  // namespace atlazio


