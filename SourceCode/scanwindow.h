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
    void scanwindow::readfile(std::string);
    ~scanwindow();

private slots:
    void on_sw_horizontalacq_radiobutton_clicked();

    void on_sw_horizontalacq_radiobutton_clicked(bool checked);

    void on_sw_verticalacq_radiobutton_clicked(bool checked);

    void on_sw_xmin_horslider_actionTriggered(int action);

    void on_sw_xmax_horslider_actionTriggered(int action);

    void on_sw_ymin_horslider_actionTriggered(int action);

    void on_sw_ymax_horslider_actionTriggered(int action);

    void on_sw_zmin_horslider_actionTriggered(int action);

    void on_sw_zmax_horslider_actionTriggered(int action);

private:
    Ui::scanwindow *ui;
};

#endif // SCANWINDOW_H
