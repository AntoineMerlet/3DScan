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

    struct params {
        float maxdepth;
        int smoothsize;
    } paramsfilt;

    struct voxelgrid {
        float x;
        float y;
        float z;
    } voxelgridfilt;

    struct bilateral {
        int sigmaS;
        float sigmaR;
    } bilateralfilt;

    struct median {
        int windowsize;
        int maxmovement;
    } medianfilt;

    struct random {
        int order;
    } randomfilt;

    struct normal {
        int order;
        int nofbins;
    } normalfilt;

    struct covar {
        int order;
    } covarfilt;

private slots:
    void on_filter_pb_clicked();

private:
    Ui::filterwindow *ui;
};

#endif // FILTERWINDOW_H
