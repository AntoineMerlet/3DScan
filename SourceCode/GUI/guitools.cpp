#include "guitools.h"

std::vector<std::string> QStringList2StdStringVec(const QStringList &list){
    std::vector<std::string> res;
    foreach( QString str, list) {
      res.push_back(str.toStdString());
    }
    return res;
}


//int loadData2DB(const QStringList &qlist, DataBase *DB){
//    std::vector<std::string> listPC = QStringList2StdStringVec(qlist);
//    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> currentPC;
//    foreach (QString str, listPC)
//    {
//        if (str.endsWith(".pcd"))
//        {
//            DB IO::loadPCD(str)
//        }
//    }
//}
