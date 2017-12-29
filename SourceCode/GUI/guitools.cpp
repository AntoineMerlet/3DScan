#include "guitools.h"


/// @author: Antoine Merlet
/// @date: 28-12-2017
/// @version 1.0
///
/// @brief Converts a QStringList to a std::vector<std::string>
std::vector<std::string> QStringList2StdStringVec(const QStringList &list){
    std::vector<std::string> res;
    foreach( QString str, list) {
        res.push_back(str.toStdString());
    }
    return res;
}


/// @author: Antoine Merlet
/// @date: 28-12-2017
/// @version 0.1
///
/// @brief Loads raw Point Clouds to the DataBase
/// @param qlist The list of path of files to load
/// @param DB the pointer to  the DataBase
/// @return The number of errors while loading
int rawPC2DB(const QStringList &qlist, DataBase *DB){
    int fails = 0;
    foreach (QString str, qlist)
    {
        if (str.endsWith(".pcd"))
            if (!DB->addRawPC(IO::loadPCD(str.toStdString())))
                fails++;
        if (str.endsWith(".ply"))
            if (!DB->addRawPC(IO::loadPLY(str.toStdString())))
                fails++;
    }
    return fails;
}

/// @author: Antoine Merlet
/// @date: 28-12-2017
/// @version 0.1
///
/// @brief Loads registered Point Clouds to the DataBase
/// @param qlist The list of path of files to load
/// @param DB the pointer to  the DataBase
/// @return The number of errors while loading
int registeredPC2DB(const QStringList &qlist, DataBase *DB){
    int fails = 0;
    foreach (QString str, qlist)
    {
        if (str.endsWith(".pcd"))
            if (!DB->addRegisteredPC(IO::loadPCD(str.toStdString())))
                fails++;
        if (str.endsWith(".ply"))
            if (!DB->addRegisteredPC(IO::loadPLY(str.toStdString())))
                fails++;
    }
    return fails;
}

/// @author: Antoine Merlet
/// @date: 28-12-2017
/// @version 0.1
///
/// @brief Loads meshs to the DataBase
/// @param qlist The list of path of files to load
/// @param DB the pointer to  the DataBase
/// @return The number of errors while loading
int meshedPC2DB(const QStringList &qlist, DataBase *DB){
    int fails = 0;
    foreach (QString str, qlist)
    {
        if (str.endsWith(".stl"))
            if (!DB->addMeshedPC(IO::loadSTL(str.toStdString())))
                fails++;
        if (str.endsWith(".vtk"))
            if (!DB->addMeshedPC(IO::loadVTK(str.toStdString())))
                fails++;
    }
    return fails;
}
