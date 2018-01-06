/****************************************************************************
** Meta object code from reading C++ file 'regwindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../SourceCode/GUI/regwindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'regwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_regwindow_t {
    QByteArrayData data[13];
    char stringdata0[204];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_regwindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_regwindow_t qt_meta_stringdata_regwindow = {
    {
QT_MOC_LITERAL(0, 0, 9), // "regwindow"
QT_MOC_LITERAL(1, 10, 9), // "updatereg"
QT_MOC_LITERAL(2, 20, 0), // ""
QT_MOC_LITERAL(3, 21, 17), // "on_p2plls_toggled"
QT_MOC_LITERAL(4, 39, 7), // "checked"
QT_MOC_LITERAL(5, 47, 14), // "on_svd_toggled"
QT_MOC_LITERAL(6, 62, 16), // "on_lmp2p_toggled"
QT_MOC_LITERAL(7, 79, 16), // "on_lmp2s_toggled"
QT_MOC_LITERAL(8, 96, 20), // "on_median_cb_toggled"
QT_MOC_LITERAL(9, 117, 21), // "on_one2one_cb_toggled"
QT_MOC_LITERAL(10, 139, 20), // "on_ransac_cb_toggled"
QT_MOC_LITERAL(11, 160, 21), // "on_surface_cb_toggled"
QT_MOC_LITERAL(12, 182, 21) // "on_reg_button_clicked"

    },
    "regwindow\0updatereg\0\0on_p2plls_toggled\0"
    "checked\0on_svd_toggled\0on_lmp2p_toggled\0"
    "on_lmp2s_toggled\0on_median_cb_toggled\0"
    "on_one2one_cb_toggled\0on_ransac_cb_toggled\0"
    "on_surface_cb_toggled\0on_reg_button_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_regwindow[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      10,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   64,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       3,    1,   65,    2, 0x08 /* Private */,
       5,    1,   68,    2, 0x08 /* Private */,
       6,    1,   71,    2, 0x08 /* Private */,
       7,    1,   74,    2, 0x08 /* Private */,
       8,    1,   77,    2, 0x08 /* Private */,
       9,    1,   80,    2, 0x08 /* Private */,
      10,    1,   83,    2, 0x08 /* Private */,
      11,    1,   86,    2, 0x08 /* Private */,
      12,    0,   89,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void, QMetaType::Bool,    4,
    QMetaType::Void, QMetaType::Bool,    4,
    QMetaType::Void, QMetaType::Bool,    4,
    QMetaType::Void, QMetaType::Bool,    4,
    QMetaType::Void, QMetaType::Bool,    4,
    QMetaType::Void, QMetaType::Bool,    4,
    QMetaType::Void, QMetaType::Bool,    4,
    QMetaType::Void, QMetaType::Bool,    4,
    QMetaType::Void,

       0        // eod
};

void regwindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        regwindow *_t = static_cast<regwindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->updatereg(); break;
        case 1: _t->on_p2plls_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 2: _t->on_svd_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 3: _t->on_lmp2p_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 4: _t->on_lmp2s_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 5: _t->on_median_cb_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 6: _t->on_one2one_cb_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 7: _t->on_ransac_cb_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 8: _t->on_surface_cb_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 9: _t->on_reg_button_clicked(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (regwindow::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&regwindow::updatereg)) {
                *result = 0;
                return;
            }
        }
    }
}

const QMetaObject regwindow::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_regwindow.data,
      qt_meta_data_regwindow,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *regwindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *regwindow::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_regwindow.stringdata0))
        return static_cast<void*>(const_cast< regwindow*>(this));
    return QDialog::qt_metacast(_clname);
}

int regwindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 10)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 10;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 10)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 10;
    }
    return _id;
}

// SIGNAL 0
void regwindow::updatereg()
{
    QMetaObject::activate(this, &staticMetaObject, 0, Q_NULLPTR);
}
QT_END_MOC_NAMESPACE
