/**
 * @file /include/atlazio/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef atlazio_QNODE_HPP_
#define atlazio_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <nav_msgs/Odometry.h>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace atlazio {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv);
	virtual ~QNode();
	bool init();
	void run() override;

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

signals:
    void loggingUpdated();
    void rosShutdown();
    void poseReceived(const double& x, const double& y);
    
private:
	int init_argc;
	char** init_argv;
	ros::Subscriber pose_subscriber;
      QStringListModel logging_model;
      
    void poseCallback(const nav_msgs::Odometry::ConstPtr& odom);
};

}  // namespace atlazio

#endif /* atlazio_QNODE_HPP_ */
