#ifndef BAG_READER_H
#define BAG_READER_H

#include <string>
#include <rosbag/bag.h>

#include <QObject>

class QCustomPlot;

namespace atlazio
{
 class BagReader : public QObject
 {
   Q_OBJECT
   
   std::string bagPath;
   rosbag::Bag bag;
   
 public:
   BagReader(QWidget* parent);
   void open(const std::string&);
   void close();
   void drawTrack(QCustomPlot*, const std::string& gpsTopic);
   bool isOpen() const;
   
 signals:
   void openingBagFile();
   void closingBagFile();

 };
  
}

#endif