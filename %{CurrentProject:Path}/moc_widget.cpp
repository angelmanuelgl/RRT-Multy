/****************************************************************************
** Meta object code from reading C++ file 'widget.h'
**
** Created by: The Qt Meta Object Compiler version 69 (Qt 6.11.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../include/widget.h"
#include <QtCore/qmetatype.h>

#include <QtCore/qtmochelpers.h>

#include <memory>


#include <QtCore/qxptype_traits.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'widget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 69
#error "This file was generated using the moc from 6.11.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

#ifndef Q_CONSTINIT
#define Q_CONSTINIT
#endif

QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
QT_WARNING_DISABLE_GCC("-Wuseless-cast")
namespace {
struct qt_meta_tag_ZN9RRTWidgetE_t {};
} // unnamed namespace

template <> constexpr inline auto RRTWidget::qt_create_metaobjectdata<qt_meta_tag_ZN9RRTWidgetE_t>()
{
    namespace QMC = QtMocConstants;
    QtMocHelpers::StringRefStorage qt_stringData {
        "RRTWidget",
        "OriginTree",
        "",
        "std::vector<float>&",
        "x",
        "y",
        "th",
        "Nrobots",
        "radio",
        "GoalTree",
        "computeVelocities",
        "Tau",
        "VelocitiesRobots",
        "Vx",
        "Vy",
        "Wang",
        "DrawMyNodes",
        "All",
        "FinalPath",
        "DistanceToTheGoal",
        "distance",
        "ParamsTreeRRT",
        "Step",
        "MaximalNodes",
        "SetTimeGrow",
        "Tgrow",
        "EulerMult",
        "DeltaT",
        "growTree"
    };

    QtMocHelpers::UintData qt_methods {
        // Slot 'OriginTree'
        QtMocHelpers::SlotData<void(std::vector<float> &, std::vector<float> &, std::vector<float> &, int, float)>(1, 2, QMC::AccessPublic, QMetaType::Void, {{
            { 0x80000000 | 3, 4 }, { 0x80000000 | 3, 5 }, { 0x80000000 | 3, 6 }, { QMetaType::Int, 7 },
            { QMetaType::Float, 8 },
        }}),
        // Slot 'GoalTree'
        QtMocHelpers::SlotData<void(std::vector<float> &, std::vector<float> &, std::vector<float> &, int, float)>(9, 2, QMC::AccessPublic, QMetaType::Void, {{
            { 0x80000000 | 3, 4 }, { 0x80000000 | 3, 5 }, { 0x80000000 | 3, 6 }, { QMetaType::Int, 7 },
            { QMetaType::Float, 8 },
        }}),
        // Slot 'computeVelocities'
        QtMocHelpers::SlotData<void(float, int)>(10, 2, QMC::AccessPublic, QMetaType::Void, {{
            { QMetaType::Float, 11 }, { QMetaType::Int, 7 },
        }}),
        // Slot 'VelocitiesRobots'
        QtMocHelpers::SlotData<void(std::vector<float> &, std::vector<float> &, std::vector<float> &, int)>(12, 2, QMC::AccessPublic, QMetaType::Void, {{
            { 0x80000000 | 3, 13 }, { 0x80000000 | 3, 14 }, { 0x80000000 | 3, 15 }, { QMetaType::Int, 7 },
        }}),
        // Slot 'DrawMyNodes'
        QtMocHelpers::SlotData<void(bool, bool)>(16, 2, QMC::AccessPublic, QMetaType::Void, {{
            { QMetaType::Bool, 17 }, { QMetaType::Bool, 18 },
        }}),
        // Slot 'DistanceToTheGoal'
        QtMocHelpers::SlotData<void(float)>(19, 2, QMC::AccessPublic, QMetaType::Void, {{
            { QMetaType::Float, 20 },
        }}),
        // Slot 'ParamsTreeRRT'
        QtMocHelpers::SlotData<void(float, int)>(21, 2, QMC::AccessPublic, QMetaType::Void, {{
            { QMetaType::Float, 22 }, { QMetaType::Int, 23 },
        }}),
        // Slot 'SetTimeGrow'
        QtMocHelpers::SlotData<void(int)>(24, 2, QMC::AccessPublic, QMetaType::Void, {{
            { QMetaType::Int, 25 },
        }}),
        // Slot 'EulerMult'
        QtMocHelpers::SlotData<void(float)>(26, 2, QMC::AccessPublic, QMetaType::Void, {{
            { QMetaType::Float, 27 },
        }}),
        // Slot 'growTree'
        QtMocHelpers::SlotData<void()>(28, 2, QMC::AccessPrivate, QMetaType::Void),
    };
    QtMocHelpers::UintData qt_properties {
    };
    QtMocHelpers::UintData qt_enums {
    };
    return QtMocHelpers::metaObjectData<RRTWidget, qt_meta_tag_ZN9RRTWidgetE_t>(QMC::MetaObjectFlag{}, qt_stringData,
            qt_methods, qt_properties, qt_enums);
}
Q_CONSTINIT const QMetaObject RRTWidget::staticMetaObject = { {
    QMetaObject::SuperData::link<QOpenGLWidget::staticMetaObject>(),
    qt_staticMetaObjectStaticContent<qt_meta_tag_ZN9RRTWidgetE_t>.stringdata,
    qt_staticMetaObjectStaticContent<qt_meta_tag_ZN9RRTWidgetE_t>.data,
    qt_static_metacall,
    nullptr,
    qt_staticMetaObjectRelocatingContent<qt_meta_tag_ZN9RRTWidgetE_t>.metaTypes,
    nullptr
} };

void RRTWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    auto *_t = static_cast<RRTWidget *>(_o);
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: _t->OriginTree((*reinterpret_cast<std::add_pointer_t<std::vector<float>&>>(_a[1])),(*reinterpret_cast<std::add_pointer_t<std::vector<float>&>>(_a[2])),(*reinterpret_cast<std::add_pointer_t<std::vector<float>&>>(_a[3])),(*reinterpret_cast<std::add_pointer_t<int>>(_a[4])),(*reinterpret_cast<std::add_pointer_t<float>>(_a[5]))); break;
        case 1: _t->GoalTree((*reinterpret_cast<std::add_pointer_t<std::vector<float>&>>(_a[1])),(*reinterpret_cast<std::add_pointer_t<std::vector<float>&>>(_a[2])),(*reinterpret_cast<std::add_pointer_t<std::vector<float>&>>(_a[3])),(*reinterpret_cast<std::add_pointer_t<int>>(_a[4])),(*reinterpret_cast<std::add_pointer_t<float>>(_a[5]))); break;
        case 2: _t->computeVelocities((*reinterpret_cast<std::add_pointer_t<float>>(_a[1])),(*reinterpret_cast<std::add_pointer_t<int>>(_a[2]))); break;
        case 3: _t->VelocitiesRobots((*reinterpret_cast<std::add_pointer_t<std::vector<float>&>>(_a[1])),(*reinterpret_cast<std::add_pointer_t<std::vector<float>&>>(_a[2])),(*reinterpret_cast<std::add_pointer_t<std::vector<float>&>>(_a[3])),(*reinterpret_cast<std::add_pointer_t<int>>(_a[4]))); break;
        case 4: _t->DrawMyNodes((*reinterpret_cast<std::add_pointer_t<bool>>(_a[1])),(*reinterpret_cast<std::add_pointer_t<bool>>(_a[2]))); break;
        case 5: _t->DistanceToTheGoal((*reinterpret_cast<std::add_pointer_t<float>>(_a[1]))); break;
        case 6: _t->ParamsTreeRRT((*reinterpret_cast<std::add_pointer_t<float>>(_a[1])),(*reinterpret_cast<std::add_pointer_t<int>>(_a[2]))); break;
        case 7: _t->SetTimeGrow((*reinterpret_cast<std::add_pointer_t<int>>(_a[1]))); break;
        case 8: _t->EulerMult((*reinterpret_cast<std::add_pointer_t<float>>(_a[1]))); break;
        case 9: _t->growTree(); break;
        default: ;
        }
    }
}

const QMetaObject *RRTWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *RRTWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_staticMetaObjectStaticContent<qt_meta_tag_ZN9RRTWidgetE_t>.strings))
        return static_cast<void*>(this);
    return QOpenGLWidget::qt_metacast(_clname);
}

int RRTWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QOpenGLWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 10)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 10;
    }
    if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 10)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 10;
    }
    return _id;
}
QT_WARNING_POP
