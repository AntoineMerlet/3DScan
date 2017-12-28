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
    std::string mes1 = "Test";
    l.Log(mes1);
    l.Log("New logg");
    l.Log("Adding a new line to the log");
    w.show();
    return a.exec();
}
