#include "guitools.h"

std::vector<std::string> QStringList2StdStringVec(const QStringList &list){
    std::vector<std::string> res;
    foreach( QString str, list) {
      res.push_back(str.toStdString());
    }
    return res;
}


int loadData2DB(const QStringList &qlist, DataBase *DB){
    foreach (QString str, qlist)
    {
        if (str.endsWith(".pcd"))
        {
            DB->addRawPC(IO::loadPCD(str.toStdString()));
        }
        if (str.endsWith(".ply"))
        {
            DB->addRawPC(IO::loadPLY(str.toStdString()));
        }
        if (str.endsWith(".stl"))
        {
            DB->addMeshedPC(IO::loadSTL(str.toStdString()));
        }
        if (str.endsWith(".vtk"))
        {
            DB->addMeshedPC(IO::loadVTK(str.toStdString()));
        }
    }
}

int rawPC2DB(const QStringList &qlist, DataBase *DB){
    foreach (QString str, qlist)
    {
        if (str.endsWith(".pcd"))
        {
            DB->addRawPC(IO::loadPCD(str.toStdString()));
        }
        if (str.endsWith(".ply"))
        {
            DB->addRawPC(IO::loadPLY(str.toStdString()));
        }
    }
}

int registeredPC2DB(const QStringList &qlist, DataBase *DB){
    foreach (QString str, qlist)
    {
        if (str.endsWith(".pcd"))
        {
            DB->addRegisteredPC(IO::loadPCD(str.toStdString()));
        }
        if (str.endsWith(".ply"))
        {
            DB->addRegisteredPC(IO::loadPLY(str.toStdString()));
        }
    }
}

int meshedPC2DB(const QStringList &qlist, DataBase *DB){
    foreach (QString str, qlist)
    {
        if (str.endsWith(".stl"))
        {
            DB->addMeshedPC(IO::loadSTL(str.toStdString()));
        }
        if (str.endsWith(".vtk"))
        {
            DB->addMeshedPC(IO::loadVTK(str.toStdString()));
        }
    }
}
