#include "regwindow.h"
#include "ui_regwindow.h"
#include "IO/logger.h"
#include <GUI/mainwindow.h>
#include <cmath>

regwindow::regwindow(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::regwindow)
{

    ui->setupUi(this);

    QObject::connect(this, SIGNAL(updatereg()), parent, SLOT(updater()));
    QObject::connect(this, SIGNAL(updateregd()), parent, SLOT(updated()));


}

regwindow::~regwindow()
{
    delete ui;
}

/// @author: Mladen Rakic
/// @date: 06-01-2018
/// @version 1.0
///
/// @brief Set of functions that enable interaction with the registering window and selection of parameters.
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
    if (checked == true){
        ui->surface_cb->setEnabled(true);
        ui->angle_sb->setEnabled(true);
    }

    if (checked == false){
        ui->one2one_cb->setChecked(false);
        ui->one2one_cb->setEnabled(false);
        ui->ransac_cb->setChecked(false);
        ui->ransac_cb->setEnabled(false);
        ui->surface_cb->setChecked(false);
        ui->surface_cb->setEnabled(false);
        ui->boundary_cb->setChecked(false);
        ui->boundary_cb->setEnabled(false);

        ui->ransacit_sb->setEnabled(false);
        ui->ransacth_sb->setEnabled(false);
    }
}

void regwindow::on_surface_cb_toggled(bool checked)
{
    if (checked == true){
        ui->boundary_cb->setEnabled(true);
    }

    if (checked == false){
        ui->one2one_cb->setChecked(false);
        ui->one2one_cb->setEnabled(false);
        ui->ransac_cb->setChecked(false);
        ui->ransac_cb->setEnabled(false);
        ui->boundary_cb->setChecked(false);
        ui->boundary_cb->setEnabled(false);

        ui->ransacit_sb->setEnabled(false);
        ui->ransacth_sb->setEnabled(false);
    }
}


void regwindow::on_boundary_cb_toggled(bool checked)
{
    if (checked == true){
        ui->one2one_cb->setEnabled(true);
    }

    if (checked == false){
        ui->ransac_cb->setChecked(false);
        ui->ransac_cb->setEnabled(false);
        ui->one2one_cb->setChecked(false);
        ui->one2one_cb->setEnabled(false);

        ui->ransacit_sb->setEnabled(false);
        ui->ransacth_sb->setEnabled(false);
    }
}

void regwindow::on_one2one_cb_toggled(bool checked)
{
    if (checked == true){
        ui->ransac_cb->setEnabled(true);
        ui->ransacit_sb->setEnabled(true);
        ui->ransacth_sb->setEnabled(true);
    }

    if (checked == false){
        ui->ransac_cb->setChecked(false);
        ui->ransac_cb->setEnabled(false);

        ui->ransacit_sb->setEnabled(false);
        ui->ransacth_sb->setEnabled(false);
    }
}




/// @author: Mladen Rakic
/// @date: 06-01-2018
/// @version 1.0
///
/// @brief Accepts the selected values for registering parameters.
void regwindow::on_reg_button_clicked()
{
    paramsreg.radius = ui->radius_sb->value();
    correspreg.maxdist = ui->dist_sb->value()/100;
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
        ransacreg.threshold = ui->ransacth_sb->value()/100;
    }

    if (ui->surface_cb->checkState()) {
        surfacereg.checked = true;
        surfacereg.angle = acos(ui->angle_sb->value() *M_PI / 180.0);
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

    this->close();
    emit updatereg();
}



void regwindow::on_default_button_clicked()
{

    this->close();
    emit updateregd();
}
