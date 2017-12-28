/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../SourceCode/GUI/mainwindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_MainWindow_t {
    QByteArrayData data[15];
    char stringdata0[411];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MainWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MainWindow_t qt_meta_stringdata_MainWindow = {
    {
QT_MOC_LITERAL(0, 0, 10), // "MainWindow"
QT_MOC_LITERAL(1, 11, 27), // "on_actionNew_scan_triggered"
QT_MOC_LITERAL(2, 39, 0), // ""
QT_MOC_LITERAL(3, 40, 38), // "on_actionImport_point_clouds_..."
QT_MOC_LITERAL(4, 79, 39), // "on_actionImport_registered_PC..."
QT_MOC_LITERAL(5, 119, 30), // "on_actionImport_mesh_triggered"
QT_MOC_LITERAL(6, 150, 38), // "on_actionExport_point_clouds_..."
QT_MOC_LITERAL(7, 189, 39), // "on_actionExport_registered_PC..."
QT_MOC_LITERAL(8, 229, 30), // "on_actionExport_mesh_triggered"
QT_MOC_LITERAL(9, 260, 36), // "on_mw_register_pc_pushbutton_..."
QT_MOC_LITERAL(10, 297, 37), // "on_mw_generatemesh_pushbutton..."
QT_MOC_LITERAL(11, 335, 24), // "on_actionAbout_triggered"
QT_MOC_LITERAL(12, 360, 30), // "on_actionUser_manual_triggered"
QT_MOC_LITERAL(13, 391, 15), // "receivedmessage"
QT_MOC_LITERAL(14, 407, 3) // "arg"

    },
    "MainWindow\0on_actionNew_scan_triggered\0"
    "\0on_actionImport_point_clouds_triggered\0"
    "on_actionImport_registered_PC_triggered\0"
    "on_actionImport_mesh_triggered\0"
    "on_actionExport_point_clouds_triggered\0"
    "on_actionExport_registered_PC_triggered\0"
    "on_actionExport_mesh_triggered\0"
    "on_mw_register_pc_pushbutton_clicked\0"
    "on_mw_generatemesh_pushbutton_clicked\0"
    "on_actionAbout_triggered\0"
    "on_actionUser_manual_triggered\0"
    "receivedmessage\0arg"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MainWindow[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      12,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   74,    2, 0x08 /* Private */,
       3,    0,   75,    2, 0x08 /* Private */,
       4,    0,   76,    2, 0x08 /* Private */,
       5,    0,   77,    2, 0x08 /* Private */,
       6,    0,   78,    2, 0x08 /* Private */,
       7,    0,   79,    2, 0x08 /* Private */,
       8,    0,   80,    2, 0x08 /* Private */,
       9,    0,   81,    2, 0x08 /* Private */,
      10,    0,   82,    2, 0x08 /* Private */,
      11,    0,   83,    2, 0x08 /* Private */,
      12,    0,   84,    2, 0x08 /* Private */,
      13,    1,   85,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   14,

       0        // eod
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        MainWindow *_t = static_cast<MainWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->on_actionNew_scan_triggered(); break;
        case 1: _t->on_actionImport_point_clouds_triggered(); break;
        case 2: _t->on_actionImport_registered_PC_triggered(); break;
        case 3: _t->on_actionImport_mesh_triggered(); break;
        case 4: _t->on_actionExport_point_clouds_triggered(); break;
        case 5: _t->on_actionExport_registered_PC_triggered(); break;
        case 6: _t->on_actionExport_mesh_triggered(); break;
        case 7: _t->on_mw_register_pc_pushbutton_clicked(); break;
        case 8: _t->on_mw_generatemesh_pushbutton_clicked(); break;
        case 9: _t->on_actionAbout_triggered(); break;
        case 10: _t->on_actionUser_manual_triggered(); break;
        case 11: _t->receivedmessage((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainWindow.data,
      qt_meta_data_MainWindow,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow.stringdata0))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 12)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 12;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 12)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 12;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
