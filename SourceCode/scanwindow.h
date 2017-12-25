#ifndef SCANWINDOW_H
#define SCANWINDOW_H

#include <QMainWindow>

namespace Ui {
class scanwindow;
}

class scanwindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit scanwindow(QWidget *parent = 0);
    ~scanwindow();

private slots:
    void on_sw_horizontalacq_radiobutton_clicked();

    void on_sw_horizontalacq_radiobutton_clicked(bool checked);

private:
    Ui::scanwindow *ui;
};

#endif // SCANWINDOW_H