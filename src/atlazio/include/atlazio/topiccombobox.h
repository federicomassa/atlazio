#ifndef TOPIC_COMBO_BOX
#define TOPIC_COMBO_BOX

#include <QComboBox>
#include <vector>
#include "topic.h"

namespace atlazio
{
  
class TopicComboBox : public QComboBox
{
  Q_OBJECT
  
  std::vector<atlazio::Topic> topics;
public:
  TopicComboBox(QWidget*);
  void addTopic(const Topic&);  
  const Topic& getTopic(const int& index) const;
  void clear();
};

}

#endif