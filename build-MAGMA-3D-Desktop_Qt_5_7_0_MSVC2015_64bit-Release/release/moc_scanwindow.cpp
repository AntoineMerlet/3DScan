/****************************************************************************
** Meta object code from reading C++ file 'scanwindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../SourceCode/GUI/scanwindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'scanwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_scanwindow_t {
    QByteArrayData data[16];
    char stringdata0[331];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_scanwindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_scanwindow_t qt_meta_stringdata_scanwindow = {
    {
QT_MOC_LITERAL(0, 0, 10), // "scanwindow"
QT_MOC_LITERAL(1, 11, 11), // "send_unhide"
QT_MOC_LITERAL(2, 23, 0), // ""
QT_MOC_LITERAL(3, 24, 39), // "on_sw_horizontalacq_radiobutt..."
QT_MOC_LITERAL(4, 64, 7), // "checked"
QT_MOC_LITERAL(5, 72, 37), // "on_sw_verticalacq_radiobutton..."
QT_MOC_LITERAL(6, 110, 34), // "on_sw_startscan_pushbutton_cl..."
QT_MOC_LITERAL(7, 145, 22), // "on_xmin_sliderReleased"
QT_MOC_LITERAL(8, 168, 33), // "on_sw_stopscan_pushbutton_cli..."
QT_MOC_LITERAL(9, 202, 19), // "on_xmin_sliderMoved"
QT_MOC_LITERAL(10, 222, 8), // "position"
QT_MOC_LITERAL(11, 231, 19), // "on_xmax_sliderMoved"
QT_MOC_LITERAL(12, 251, 19), // "on_ymin_sliderMoved"
QT_MOC_LITERAL(13, 271, 19), // "on_ymax_sliderMoved"
QT_MOC_LITERAL(14, 291, 19), // "on_zmin_sliderMoved"
QT_MOC_LITERAL(15, 311, 19) // "on_zmax_sliderMoved"

    },
    "scanwindow\0send_unhide\0\0"
    "on_sw_horizontalacq_radiobutton_clicked\0"
    "checked\0on_sw_verticalacq_radiobutton_clicked\0"
    "on_sw_startscan_pushbutton_clicked\0"
    "on_xmin_sliderReleased\0"
    "on_sw_stopscan_pushbutton_clicked\0"
    "on_xmin_sliderMoved\0position\0"
    "on_xmax_sliderMoved\0on_ymin_sliderMoved\0"
    "on_ymax_sliderMoved\0on_zmin_sliderMoved\0"
    "on_zmax_sliderMoved"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_scanwindow[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      13,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   79,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       3,    0,   80,    2, 0x08 /* Private */,
       3,    1,   81,    2, 0x08 /* Private */,
       5,    1,   84,    2, 0x08 /* Private */,
       6,    0,   87,    2, 0x08 /* Private */,
       7,    0,   88,    2, 0x08 /* Private */,
       8,    0,   89,    2, 0x08 /* Private */,
       9,    1,   90,    2, 0x08 /* Private */,
      11,    1,   93,    2, 0x08 /* Private */,
      12,    1,   96,    2, 0x08 /* Private */,
      13,    1,   99,    2, 0x08 /* Private */,
      14,    1,  102,    2, 0x08 /* Private */,
      15,    1,  105,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,    4,
    QMetaType::Void, QMetaType::Bool,    4,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   10,
    QMetaType::Void, QMetaType::Int,   10,
    QMetaType::Void, QMetaType::Int,   10,
    QMetaType::Void, QMetaType::Int,   10,
    QMetaType::Void, QMetaType::Int,   10,
    QMetaType::Void, QMetaType::Int,   10,

       0        // eod
};

void scanwindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        scanwindow *_t = static_cast<scanwindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->send_unhide(); break;
        case 1: _t->on_sw_horizontalacq_radiobutton_clicked(); break;
        case 2: _t->on_sw_horizontalacq_radiobutton_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 3: _t->on_sw_verticalacq_radiobutton_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 4: _t->on_sw_startscan_pushbutton_clicked(); break;
        case 5: _t->on_xmin_sliderReleased(); break;
        case 6: _t->on_sw_stopscan_pushbutton_clicked(); break;
        case 7: _t->on_xmin_sliderMoved((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 8: _t->on_xmax_sliderMoved((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 9: _t->on_ymin_sliderMoved((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 10: _t->on_ymax_sliderMoved((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 11: _t->on_zmin_sliderMoved((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 12: _t->on_zmax_sliderMoved((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (scanwindow::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&scanwindow::send_unhide)) {
                *result = 0;
                return;
            }
        }
    }
}

const QMetaObject scanwindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_scanwindow.data,
      qt_meta_data_scanwindow,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *scanwindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *scanwindow::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_scanwindow.stringdata0))
        return static_cast<void*>(const_cast< scanwindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int scanwindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 13)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 13;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 13)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 13;
    }
    return _id;
}

// SIGNAL 0
void scanwindow::send_unhide()
{
    QMetaObject::activate(this, &staticMetaObject, 0, Q_NULLPTR);
}
QT_END_MOC_NAMESPACE
