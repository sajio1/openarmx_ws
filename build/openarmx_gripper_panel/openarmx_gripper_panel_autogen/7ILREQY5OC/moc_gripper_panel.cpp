/****************************************************************************
** Meta object code from reading C++ file 'gripper_panel.hpp'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../src/openarmx_tools/openarmx_gripper_panel/include/openarmx_gripper_panel/gripper_panel.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'gripper_panel.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_openarmx_gripper_panel__GripperPanel_t {
    QByteArrayData data[10];
    char stringdata0[150];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_openarmx_gripper_panel__GripperPanel_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_openarmx_gripper_panel__GripperPanel_t qt_meta_stringdata_openarmx_gripper_panel__GripperPanel = {
    {
QT_MOC_LITERAL(0, 0, 36), // "openarmx_gripper_panel::Gripp..."
QT_MOC_LITERAL(1, 37, 14), // "onCloseClicked"
QT_MOC_LITERAL(2, 52, 0), // ""
QT_MOC_LITERAL(3, 53, 13), // "onHalfClicked"
QT_MOC_LITERAL(4, 67, 13), // "onOpenClicked"
QT_MOC_LITERAL(5, 81, 14), // "onApplyClicked"
QT_MOC_LITERAL(6, 96, 15), // "onSliderChanged"
QT_MOC_LITERAL(7, 112, 5), // "value"
QT_MOC_LITERAL(8, 118, 25), // "onGripperSelectionChanged"
QT_MOC_LITERAL(9, 144, 5) // "index"

    },
    "openarmx_gripper_panel::GripperPanel\0"
    "onCloseClicked\0\0onHalfClicked\0"
    "onOpenClicked\0onApplyClicked\0"
    "onSliderChanged\0value\0onGripperSelectionChanged\0"
    "index"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_openarmx_gripper_panel__GripperPanel[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   44,    2, 0x08 /* Private */,
       3,    0,   45,    2, 0x08 /* Private */,
       4,    0,   46,    2, 0x08 /* Private */,
       5,    0,   47,    2, 0x08 /* Private */,
       6,    1,   48,    2, 0x08 /* Private */,
       8,    1,   51,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    7,
    QMetaType::Void, QMetaType::Int,    9,

       0        // eod
};

void openarmx_gripper_panel::GripperPanel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<GripperPanel *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->onCloseClicked(); break;
        case 1: _t->onHalfClicked(); break;
        case 2: _t->onOpenClicked(); break;
        case 3: _t->onApplyClicked(); break;
        case 4: _t->onSliderChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 5: _t->onGripperSelectionChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject openarmx_gripper_panel::GripperPanel::staticMetaObject = { {
    QMetaObject::SuperData::link<rviz_common::Panel::staticMetaObject>(),
    qt_meta_stringdata_openarmx_gripper_panel__GripperPanel.data,
    qt_meta_data_openarmx_gripper_panel__GripperPanel,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *openarmx_gripper_panel::GripperPanel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *openarmx_gripper_panel::GripperPanel::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_openarmx_gripper_panel__GripperPanel.stringdata0))
        return static_cast<void*>(this);
    return rviz_common::Panel::qt_metacast(_clname);
}

int openarmx_gripper_panel::GripperPanel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = rviz_common::Panel::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 6)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 6;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
