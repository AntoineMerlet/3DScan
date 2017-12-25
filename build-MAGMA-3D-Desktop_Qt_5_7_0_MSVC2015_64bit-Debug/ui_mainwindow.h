/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.7.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionNew_scan;
    QAction *actionImport_point_clouds;
    QAction *actionImport_registered_PC;
    QAction *actionImport_mesh;
    QAction *actionExport_point_clouds;
    QAction *actionExport_registered_PC;
    QAction *actionExport_mesh;
    QAction *actionUser_manual;
    QAction *actionAbout;
    QWidget *centralWidget;
    QTabWidget *mw_explorer_tabwidget;
    QWidget *mw_pc_tab;
    QWidget *mw_registeredpc_tab;
    QWidget *mw_mesh_tab;
    QGraphicsView *mw_visualizer_graphicsview;
    QFrame *mw_register_frame;
    QPushButton *mw_register_pc_pushbutton;
    QPushButton *mw_generatemesh_pushbutton;
    QLabel *mw_visualizer_label;
    QLabel *mw_pcexplorer_label;
    QTextEdit *mw_logger_textedit;
    QMenuBar *menuBar;
    QMenu *menuMAGMA_3D_scanner;
    QMenu *menuImport;
    QMenu *menuExport;
    QMenu *menuHelp;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(650, 500);
        actionNew_scan = new QAction(MainWindow);
        actionNew_scan->setObjectName(QStringLiteral("actionNew_scan"));
        actionImport_point_clouds = new QAction(MainWindow);
        actionImport_point_clouds->setObjectName(QStringLiteral("actionImport_point_clouds"));
        actionImport_registered_PC = new QAction(MainWindow);
        actionImport_registered_PC->setObjectName(QStringLiteral("actionImport_registered_PC"));
        actionImport_mesh = new QAction(MainWindow);
        actionImport_mesh->setObjectName(QStringLiteral("actionImport_mesh"));
        actionExport_point_clouds = new QAction(MainWindow);
        actionExport_point_clouds->setObjectName(QStringLiteral("actionExport_point_clouds"));
        actionExport_registered_PC = new QAction(MainWindow);
        actionExport_registered_PC->setObjectName(QStringLiteral("actionExport_registered_PC"));
        actionExport_mesh = new QAction(MainWindow);
        actionExport_mesh->setObjectName(QStringLiteral("actionExport_mesh"));
        actionUser_manual = new QAction(MainWindow);
        actionUser_manual->setObjectName(QStringLiteral("actionUser_manual"));
        actionAbout = new QAction(MainWindow);
        actionAbout->setObjectName(QStringLiteral("actionAbout"));
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        mw_explorer_tabwidget = new QTabWidget(centralWidget);
        mw_explorer_tabwidget->setObjectName(QStringLiteral("mw_explorer_tabwidget"));
        mw_explorer_tabwidget->setGeometry(QRect(20, 40, 191, 181));
        mw_pc_tab = new QWidget();
        mw_pc_tab->setObjectName(QStringLiteral("mw_pc_tab"));
        mw_explorer_tabwidget->addTab(mw_pc_tab, QString());
        mw_registeredpc_tab = new QWidget();
        mw_registeredpc_tab->setObjectName(QStringLiteral("mw_registeredpc_tab"));
        mw_explorer_tabwidget->addTab(mw_registeredpc_tab, QString());
        mw_mesh_tab = new QWidget();
        mw_mesh_tab->setObjectName(QStringLiteral("mw_mesh_tab"));
        mw_explorer_tabwidget->addTab(mw_mesh_tab, QString());
        mw_visualizer_graphicsview = new QGraphicsView(centralWidget);
        mw_visualizer_graphicsview->setObjectName(QStringLiteral("mw_visualizer_graphicsview"));
        mw_visualizer_graphicsview->setGeometry(QRect(230, 40, 391, 331));
        mw_register_frame = new QFrame(centralWidget);
        mw_register_frame->setObjectName(QStringLiteral("mw_register_frame"));
        mw_register_frame->setGeometry(QRect(40, 240, 151, 121));
        mw_register_frame->setFrameShape(QFrame::Box);
        mw_register_frame->setFrameShadow(QFrame::Raised);
        mw_register_pc_pushbutton = new QPushButton(mw_register_frame);
        mw_register_pc_pushbutton->setObjectName(QStringLiteral("mw_register_pc_pushbutton"));
        mw_register_pc_pushbutton->setGeometry(QRect(20, 20, 111, 31));
        mw_generatemesh_pushbutton = new QPushButton(mw_register_frame);
        mw_generatemesh_pushbutton->setObjectName(QStringLiteral("mw_generatemesh_pushbutton"));
        mw_generatemesh_pushbutton->setGeometry(QRect(20, 70, 111, 31));
        mw_visualizer_label = new QLabel(centralWidget);
        mw_visualizer_label->setObjectName(QStringLiteral("mw_visualizer_label"));
        mw_visualizer_label->setGeometry(QRect(230, 20, 61, 16));
        mw_pcexplorer_label = new QLabel(centralWidget);
        mw_pcexplorer_label->setObjectName(QStringLiteral("mw_pcexplorer_label"));
        mw_pcexplorer_label->setGeometry(QRect(30, 20, 101, 16));
        mw_logger_textedit = new QTextEdit(centralWidget);
        mw_logger_textedit->setObjectName(QStringLiteral("mw_logger_textedit"));
        mw_logger_textedit->setGeometry(QRect(20, 380, 601, 81));
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 650, 21));
        menuMAGMA_3D_scanner = new QMenu(menuBar);
        menuMAGMA_3D_scanner->setObjectName(QStringLiteral("menuMAGMA_3D_scanner"));
        menuImport = new QMenu(menuBar);
        menuImport->setObjectName(QStringLiteral("menuImport"));
        menuExport = new QMenu(menuBar);
        menuExport->setObjectName(QStringLiteral("menuExport"));
        menuHelp = new QMenu(menuBar);
        menuHelp->setObjectName(QStringLiteral("menuHelp"));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);

        menuBar->addAction(menuMAGMA_3D_scanner->menuAction());
        menuBar->addAction(menuImport->menuAction());
        menuBar->addAction(menuExport->menuAction());
        menuBar->addAction(menuHelp->menuAction());
        menuMAGMA_3D_scanner->addAction(actionNew_scan);
        menuImport->addAction(actionImport_point_clouds);
        menuImport->addAction(actionImport_registered_PC);
        menuImport->addAction(actionImport_mesh);
        menuExport->addAction(actionExport_point_clouds);
        menuExport->addAction(actionExport_registered_PC);
        menuExport->addAction(actionExport_mesh);
        menuHelp->addAction(actionUser_manual);
        menuHelp->addAction(actionAbout);

        retranslateUi(MainWindow);

        mw_explorer_tabwidget->setCurrentIndex(2);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0));
        actionNew_scan->setText(QApplication::translate("MainWindow", "New scan", 0));
        actionImport_point_clouds->setText(QApplication::translate("MainWindow", "Import point clouds", 0));
        actionImport_registered_PC->setText(QApplication::translate("MainWindow", "Import registered PC", 0));
        actionImport_mesh->setText(QApplication::translate("MainWindow", "Import mesh", 0));
        actionExport_point_clouds->setText(QApplication::translate("MainWindow", "Export point clouds", 0));
        actionExport_registered_PC->setText(QApplication::translate("MainWindow", "Export registered PC", 0));
        actionExport_mesh->setText(QApplication::translate("MainWindow", "Export mesh", 0));
        actionUser_manual->setText(QApplication::translate("MainWindow", "User manual", 0));
        actionAbout->setText(QApplication::translate("MainWindow", "About", 0));
        mw_explorer_tabwidget->setTabText(mw_explorer_tabwidget->indexOf(mw_pc_tab), QApplication::translate("MainWindow", "PC", 0));
        mw_explorer_tabwidget->setTabText(mw_explorer_tabwidget->indexOf(mw_registeredpc_tab), QApplication::translate("MainWindow", "Registered PC", 0));
        mw_explorer_tabwidget->setTabText(mw_explorer_tabwidget->indexOf(mw_mesh_tab), QApplication::translate("MainWindow", "Mesh", 0));
        mw_register_pc_pushbutton->setText(QApplication::translate("MainWindow", "Register point clouds", 0));
        mw_generatemesh_pushbutton->setText(QApplication::translate("MainWindow", "Generate mesh", 0));
        mw_visualizer_label->setText(QApplication::translate("MainWindow", "Visualizer", 0));
        mw_pcexplorer_label->setText(QApplication::translate("MainWindow", "Point cloud explorer", 0));
        menuMAGMA_3D_scanner->setTitle(QApplication::translate("MainWindow", "Scan", 0));
        menuImport->setTitle(QApplication::translate("MainWindow", "Import", 0));
        menuExport->setTitle(QApplication::translate("MainWindow", "Export", 0));
        menuHelp->setTitle(QApplication::translate("MainWindow", "Help", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H