#include "regwindow.h"
#include "ui_regwindow.h"

regwindow::regwindow(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::regwindow)
{

    ui->setupUi(this);

    QObject::connect(this, SIGNAL(updatereg()), parent, SLOT(updater()));

}

regwindow::~regwindow()
{
    delete ui;
}

void regwindow::on_p2plls_toggled(bool checked)
{
    ui->svd->setChecked(false);
    ui->lmp2p->setChecked(false);
    ui->lmp2s->setChecked(false);
    methodreg.type = 1;
}

void regwindow::on_svd_toggled(bool checked)
{
    ui->p2plls->setChecked(false);
    ui->lmp2p->setChecked(false);
    ui->lmp2s->setChecked(false);
    methodreg.type = 2;
}

void regwindow::on_lmp2p_toggled(bool checked)
{
    ui->p2plls->setChecked(false);
    ui->svd->setChecked(false);
    ui->lmp2s->setChecked(false);
    methodreg.type = 3;
}

void regwindow::on_lmp2s_toggled(bool checked)
{
    ui->p2plls->setChecked(false);
    ui->svd->setChecked(false);
    ui->lmp2p->setChecked(false);
    methodreg.type = 4;
}



void regwindow::on_median_cb_toggled(bool checked)
{
    ui->one2one_cb->setEnabled(true);
}


void regwindow::on_one2one_cb_toggled(bool checked)
{
    ui->ransac_cb->setEnabled(true);
    ui->ransacit_sb->setEnabled(true);
    ui->ransacth_sb->setEnabled(true);
}



void regwindow::on_ransac_cb_toggled(bool checked)
{
    ui->surface_cb->setEnabled(true);
    ui->angle_sb->setEnabled(true);
}



void regwindow::on_surface_cb_toggled(bool checked)
{
    ui->boundary_cb->setEnabled(true);
}

void regwindow::on_reg_button_clicked()
{
    paramsreg.maxdepth = ui->depth_sb->value();
    paramsreg.smoothsize = ui->smooth_sb->value();

    correspreg.maxdist = ui->dist_sb->value();
    methodreg.maxiter = ui->maxiter_sb->value();

    if (ui->median_cb->checkState()) {
        mediandistreg.checked = true;
        mediandistreg.medfact = ui->medianfact_sb->value();
    }

    if (ui->one2one_cb->checkState()) {
        one2onereg.checked = true;
    }

    if (ui->ransac_cb->checkState()) {
        ransacreg.checked = true;
        ransacreg.iter = ui->ransacit_sb->value();
        ransacreg.threshold = ui->ransacth_sb->value();
    }

    if (ui->surface_cb->checkState()) {
        surfacereg.checked = true;
        surfacereg.angle = ui->angle_sb->value();
    }

    if (ui->boundary_cb->checkState()) {
        boundaryreg.checked = true;
    }

    if (ui->similar_cb->checkState()) {
        similarreg.checked = true;
        similarreg.iter = ui->similariter_sb->value();
    }

    if (ui->relativemse_cb->checkState()) {
        relativereg.checked = true;
        relativereg.mse = ui->relativemse_sb->value();
    }

    if (ui->absolutemse_cb->checkState()) {
        absolutereg.checked = true;
        absolutereg.mse = ui->absolutemse_sb->value();
    }

    emit updatereg();
    this->close();

}
