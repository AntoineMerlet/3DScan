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

    struct params {
        float maxdepth;
        int smoothsize;
    } paramsreg;

    struct corresp {
        float maxdist;
    } correspreg;

    struct method {
        int type;
        int maxiter;
    } methodreg;

    struct similar {
        bool checked = false;
        int iter;
    } similarreg;

    struct relative {
        bool checked = false;
        float mse;
    } relativereg;

    struct absolute {
        bool checked = false;
        float mse;
    } absolutereg;

    struct mediandist {
        bool checked = false;
        float medfact;
    } mediandistreg;

    struct one2one {
        bool checked = false;
    } one2onereg;

    struct ransac {
        bool checked = false;
        int iter;
        float threshold;
    } ransacreg;

    struct surface {
        bool checked = false;
        int angle;
    } surfacereg;

    struct boundary {
        bool checked = false;
    } boundaryreg;


private slots:
    void on_p2plls_toggled(bool checked);

    void on_svd_toggled(bool checked);

    void on_lmp2p_toggled(bool checked);

    void on_lmp2s_toggled(bool checked);

    void on_median_cb_toggled(bool checked);

    void on_one2one_cb_toggled(bool checked);

    void on_ransac_cb_toggled(bool checked);

    void on_surface_cb_toggled(bool checked);

    void on_reg_button_clicked();

private:
    Ui::regwindow *ui;

signals:
    void updatereg();
};

#endif // REGWINDOW_H
