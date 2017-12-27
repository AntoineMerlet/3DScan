#include "GUI/mainwindow.h"
#include <QApplication>
#include <logger.h>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    logger l;
    l.CreateLog();
    QObject::connect(&l, SIGNAL(sendmessage(QString)), &w, SLOT(receivedmessage(QString)));
    w.show();

    return a.exec();
}
