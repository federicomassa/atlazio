#include "topiccombobox.h"

namespace atlazio {
  
TopicComboBox::TopicComboBox(QWidget* parent) : QComboBox(parent)
{
}
 
void TopicComboBox::addTopic(const Topic& t)
{
  int index = topics.size();
  topics.push_back(t);
  
  insertItem(index, QString(t.getName() + " (" + t.getType() + ")"));
}

void TopicComboBox::clear()
{
  // Clear vector and parent QComboBox
  for (int i = 0; i < topics.size(); i++)
    removeItem(i);
  
  topics.clear();  
}

const Topic& TopicComboBox::getTopic(const int& index) const
{
  return topics.at(index);
}

  
}