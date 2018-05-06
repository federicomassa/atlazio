#include "topic.h"

namespace atlazio {
  
Topic::Topic(const QString& n, const QString& t)
{
  name = n;
  datatype = t;
}

const QString& Topic::getName() const
{
  return name;
}

const QString& Topic::getType() const
{
  return datatype;
}


}
