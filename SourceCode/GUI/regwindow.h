#ifndef REGWINDOW_H
#define REGWINDOW_H

#include <QDialog>

namespace Ui {
class regwindow;
}

class regwindow : public QDialog
{
    Q_OBJECT

public:
    explicit regwindow(QWidget *parent = 0);
    ~regwindow();

private:
    Ui::regwindow *ui;
};

#endif // REGWINDOW_H
