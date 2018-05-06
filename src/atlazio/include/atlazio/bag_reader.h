#ifndef BAG_READER_H
#define BAG_READER_H

#include <string>
#include <rosbag/bag.h>

#include <QObject>
#include <QMap>
#include <QString>

class QCustomPlot;

namespace atlazio
{
 class BagReader : public QObject
 {
   Q_OBJECT
   
   std::string bagPath;
   rosbag::Bag bag;
   
   // Track plot range
   double trackMinX;
   double trackMaxX;
   double trackMinY;
   double trackMaxY;
   
   // Margin left (in %)
   double graphMargin;
   
   
 public:
   BagReader(QWidget* parent);
   void open(const std::string&);
   void close();
   void drawTrack(QCustomPlot*, const std::string& gpsTopic);
   bool isOpen() const;
   QMap<QString, QString> getAvailableTopics() const;
   
 signals:
   void openingBagFile();
   void closingBagFile();
   
 };
  
}

#endif