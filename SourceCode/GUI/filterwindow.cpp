#include "filterwindow.h"
#include "ui_filterwindow.h"
#include "GUI/mainwindow.h"

filterwindow::filterwindow(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::filterwindow)
{
    ui->setupUi(this);

    QObject::connect(this, SIGNAL(updatefilt()), parent, SLOT(updatef()));
}

filterwindow::~filterwindow()
{
    delete ui;
}

/// @author: Mladen Rakic
/// @date: 06-01-2018
/// @version 1.0
///
/// @brief Accepts the selected values for filtering parameters.
void filterwindow::on_filter_pb_clicked()
{
    paramsfilt.maxdepth = ui->depth_sb->value()/100;
    paramsfilt.smoothsize = ui->smooth_sb->value();
    if (ui->voxelgrid_cb->checkState()) {
        voxelgridfilt.x = ui->x_sb->value()/100;
        voxelgridfilt.y = ui->y_sb->value()/100;
        voxelgridfilt.z = ui->z_sb->value()/100;
        voxelgridfilt.checked = true;
    }

    if (ui->bilateral_cb->checkState()) {
        bilateralfilt.sigmaS = ui->sigmas_sb->value();
        bilateralfilt.sigmaR = ui->sigmar_sb->value()/100;
        bilateralfilt.checked = true;
    }

    if (ui->median_cb->checkState()) {
        medianfilt.windowsize = ui->windowsize_sb->value();
        medianfilt.maxmovement = ui->maxmov_sb->value()/100;
        medianfilt.checked = true;
    }

    if (ui->random_cb->checkState()) {
        randomfilt.order = ui->randord_sb->value();
        randomfilt.checked = true;
    }

    if (ui->normal_cb->checkState()) {
        normalfilt.order = ui->normord_sb->value();
        normalfilt.nofbins = ui->normbins_sb->value();
        normalfilt.checked = true;
    }

    if (ui->covar_cb->checkState()) {
        covarfilt.order = ui->covarord_sb->value();
        covarfilt.checked = true;
    }

    this->close();
    emit updatefilt();
}
