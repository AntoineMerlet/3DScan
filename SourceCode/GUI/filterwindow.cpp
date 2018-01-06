#include "filterwindow.h"
#include "ui_filterwindow.h"

filterwindow::filterwindow(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::filterwindow)
{
    ui->setupUi(this);
}

filterwindow::~filterwindow()
{
    delete ui;
}

void filterwindow::on_filter_pb_clicked()
{
    params.maxdepth = ui->depth_sb->value();
    params.smoothsize = ui->smooth_sb->value();

    if (ui->voxelgrid_cb->checkState() == true) {
        voxelgridfilt.x = ui->x_sb->value();
        voxelgridfilt.y = ui->y_sb->value();
        voxelgridfilt.z = ui->z_sb->value();
        voxelgridfilt.checked = true;
    }

    if (ui->bilateral_cb->checkState() == true) {
        bilateralfilt.sigmaS = ui->sigmas_sb->value();
        bilateralfilt.sigmaR = ui->sigmar_sb->value();
        bilateralfilt.checked = true;
    }

    if (ui->median_cb->checkState() == true) {
        medianfilt.windowsize = ui->windowsize_sb->value();
        medianfilt.maxmovement = ui->maxmov_sb->value();
        medianfilt.checked = true;
    }

    if (ui->random_cb->checkState() == true) {
        randomfilt.order = ui->randord_sb->value();
        randomfilt.checked = true;
    }

    if (ui->normal_cb->checkState() == true) {
        normalfilt.order = ui->normord_sb->value();
        normalfilt.nofbins = ui->normbins_sb->value();
        normalfilt.checked = true;
    }

    if (ui->covar_cb->checkState() == true) {
        covarfilt.order = ui->covarord_sb->value();
        covarfilt.checked = true;
    }

    this->close();
}
