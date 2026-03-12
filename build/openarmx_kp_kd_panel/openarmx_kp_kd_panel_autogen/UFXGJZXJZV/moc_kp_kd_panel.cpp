/****************************************************************************
** Meta object code from reading C++ file 'kp_kd_panel.hpp'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../src/openarmx_tools/openarmx_kp_kd_panel/include/openarmx_kp_kd_panel/kp_kd_panel.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'kp_kd_panel.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_openarmx_kp_kd_panel__KpKdPanel_t {
    QByteArrayData data[14];
    char stringdata0[228];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_openarmx_kp_kd_panel__KpKdPanel_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_openarmx_kp_kd_panel__KpKdPanel_t qt_meta_stringdata_openarmx_kp_kd_panel__KpKdPanel = {
    {
QT_MOC_LITERAL(0, 0, 31), // "openarmx_kp_kd_panel::KpKdPanel"
QT_MOC_LITERAL(1, 32, 17), // "onKpSliderChanged"
QT_MOC_LITERAL(2, 50, 0), // ""
QT_MOC_LITERAL(3, 51, 5), // "value"
QT_MOC_LITERAL(4, 57, 17), // "onKdSliderChanged"
QT_MOC_LITERAL(5, 75, 24), // "onGripperKpSliderChanged"
QT_MOC_LITERAL(6, 100, 24), // "onGripperKdSliderChanged"
QT_MOC_LITERAL(7, 125, 14), // "onApplyClicked"
QT_MOC_LITERAL(8, 140, 12), // "onResetArmKp"
QT_MOC_LITERAL(9, 153, 12), // "onResetArmKd"
QT_MOC_LITERAL(10, 166, 16), // "onResetGripperKp"
QT_MOC_LITERAL(11, 183, 16), // "onResetGripperKd"
QT_MOC_LITERAL(12, 200, 21), // "onArmSelectionChanged"
QT_MOC_LITERAL(13, 222, 5) // "index"

    },
    "openarmx_kp_kd_panel::KpKdPanel\0"
    "onKpSliderChanged\0\0value\0onKdSliderChanged\0"
    "onGripperKpSliderChanged\0"
    "onGripperKdSliderChanged\0onApplyClicked\0"
    "onResetArmKp\0onResetArmKd\0onResetGripperKp\0"
    "onResetGripperKd\0onArmSelectionChanged\0"
    "index"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_openarmx_kp_kd_panel__KpKdPanel[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      10,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   64,    2, 0x08 /* Private */,
       4,    1,   67,    2, 0x08 /* Private */,
       5,    1,   70,    2, 0x08 /* Private */,
       6,    1,   73,    2, 0x08 /* Private */,
       7,    0,   76,    2, 0x08 /* Private */,
       8,    0,   77,    2, 0x08 /* Private */,
       9,    0,   78,    2, 0x08 /* Private */,
      10,    0,   79,    2, 0x08 /* Private */,
      11,    0,   80,    2, 0x08 /* Private */,
      12,    1,   81,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   13,

       0        // eod
};

void openarmx_kp_kd_panel::KpKdPanel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<KpKdPanel *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->onKpSliderChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->onKdSliderChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->onGripperKpSliderChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: _t->onGripperKdSliderChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: _t->onApplyClicked(); break;
        case 5: _t->onResetArmKp(); break;
        case 6: _t->onResetArmKd(); break;
        case 7: _t->onResetGripperKp(); break;
        case 8: _t->onResetGripperKd(); break;
        case 9: _t->onArmSelectionChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject openarmx_kp_kd_panel::KpKdPanel::staticMetaObject = { {
    QMetaObject::SuperData::link<rviz_common::Panel::staticMetaObject>(),
    qt_meta_stringdata_openarmx_kp_kd_panel__KpKdPanel.data,
    qt_meta_data_openarmx_kp_kd_panel__KpKdPanel,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *openarmx_kp_kd_panel::KpKdPanel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *openarmx_kp_kd_panel::KpKdPanel::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_openarmx_kp_kd_panel__KpKdPanel.stringdata0))
        return static_cast<void*>(this);
    return rviz_common::Panel::qt_metacast(_clname);
}

int openarmx_kp_kd_panel::KpKdPanel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = rviz_common::Panel::qt_metacall(_c, _id, _a);
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
QT_WARNING_POP
QT_END_MOC_NAMESPACE
