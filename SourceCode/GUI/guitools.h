#ifndef GUITOOLS_H
#define GUITOOLS_H

#include <vector>
#include <string>
#include <QStringList>
#include "IO/iotools.h"
#include "Storage/database.h"

std::vector<std::string> QStringList2StdStringVec(const QStringList &);

//int loadData2DB(const QStringList &);
#endif // GUITOOLS_H
