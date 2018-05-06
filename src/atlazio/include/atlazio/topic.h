#ifndef TOPIC_H
#define TOPIC_H

#include <QString>

namespace atlazio
{

class Topic
{
public:
  Topic(const QString&, const QString&);
  const QString& getName() const;
  const QString& getType() const;
private:
  QString name;
  QString datatype;
};

}
#endif