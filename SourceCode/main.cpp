#include "GUI/mainwindow.h"
#include <QApplication>
#include <IO/logger.h>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    INITLOG; // Initiliazing the logger
    QObject::connect(logger::LogPointer, SIGNAL(sendmessage(QString)),
                     &w, SLOT(receivedmessage(QString))); // Connecting logger and MainWindow
    LOG("Program initilized");
    w.setWindowTitle("MAGMA Project");
    w.setWindowIcon(QIcon("logo_triangle_small.ico"));
    w.show();
    return a.exec();
}
