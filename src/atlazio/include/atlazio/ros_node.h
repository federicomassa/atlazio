/**/
/*****************************************************************************
** Node that 
*****************************************************************************/

#ifndef ROS_NODE_H
#define ROS_NODE_H

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#include <string>
#include <QThread>
#include <QStringListModel>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace atlazio {

/*****************************************************************************
** Class
*****************************************************************************/

class RosNode : public QThread {
    Q_OBJECT
public:
	RosNode(int argc, char** argv);
	virtual ~RosNode();
	bool init(const QString&, const QString&);
	void run() override;
	const std::vector<std::string>& getAvailableTopics() const;

signals:
  void newXYPoint(const double&, const double&);
	
public slots:
  void changeSubscription(const QString& topicName, const QString& topicType);
  void slot2();
    
private:
	int init_argc;
	char** init_argv;
      
	QString currentTopicName;
	QString currentTopicType;
	
	void odometryCallback(const nav_msgs::Odometry::ConstPtr&);
	void imuCallback(const sensor_msgs::Imu::ConstPtr&);

	ros::AsyncSpinner spinner ;
	ros::NodeHandle n;
	ros::Subscriber sub;
;
};

}  // namespace atlazio

#endif /* ROS_MONITOR_H */