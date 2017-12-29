#ifndef GUITOOLS_H
#define GUITOOLS_H

#include <vector>
#include <string>
#include <QStringList>
#include "IO/iotools.h"
#include "Storage/database.h"

std::vector<std::string> QStringList2StdStringVec(const QStringList &);

int meshedPC2DB(const QStringList &, DataBase *);
int rawPC2DB(const QStringList &, DataBase *);
int registeredPC2DB(const QStringList &, DataBase *);
#endif // GUITOOLS_H
