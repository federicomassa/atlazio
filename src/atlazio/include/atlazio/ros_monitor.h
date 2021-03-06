/**/
/*****************************************************************************
** Node that 
*****************************************************************************/

#ifndef ROS_MONITOR_H
#define ROS_MONITOR_H

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
#include <QMap>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace atlazio {

/*****************************************************************************
** Class
*****************************************************************************/

class RosMonitor : public QThread {
    Q_OBJECT
public:
	RosMonitor(int argc, char** argv);
	virtual ~RosMonitor();
	bool init();
	void run() override;
	const QMap<QString, QString>& getAvailableTopics() const;

private:
	QMap<QString, QString> availableTopics;
	  
signals:
    void rosShutdown();
    
private:
	int init_argc;
	char** init_argv;
      
};

}  // namespace atlazio

#endif /* ROS_MONITOR_H */