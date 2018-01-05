#ifndef FILTERWINDOW_H
#define FILTERWINDOW_H

#include <QDialog>

namespace Ui {
class filterwindow;
}

class filterwindow : public QDialog
{
    Q_OBJECT

public:
    explicit filterwindow(QWidget *parent = 0);
    ~filterwindow();

private:
    Ui::filterwindow *ui;
};

#endif // FILTERWINDOW_H
