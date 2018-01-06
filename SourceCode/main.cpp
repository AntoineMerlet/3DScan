#include "GUI/mainwindow.h"
#include <QApplication>
#include <logger.h>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    INITLOG; // Initiliazing the logger
    QObject::connect(logger::LogPointer, SIGNAL(sendmessage(QString)),
                     &w, SLOT(receivedmessage(QString))); // Connecting logger and MainWindow
<<<<<<< HEAD

    LOG("Program initilized");
=======
    LOG("Program initialized");
>>>>>>> 5e8699a1f4866001dff3667338b238cd30a8685b
    w.setWindowTitle("MAGMA Project");
    w.setWindowIcon(QIcon("logo_triangle_small.ico"));
    w.show();
    return a.exec();
}
