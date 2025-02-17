/****************************************************************************
** Meta object code from reading C++ file 'witmotion_ros.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.13)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../witmotion_ros/include/witmotion_ros.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'witmotion_ros.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.13. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_ROSWitmotionSensorController_t {
    QByteArrayData data[11];
    char stringdata0[147];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_ROSWitmotionSensorController_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_ROSWitmotionSensorController_t qt_meta_stringdata_ROSWitmotionSensorController = {
    {
QT_MOC_LITERAL(0, 0, 28), // "ROSWitmotionSensorController"
QT_MOC_LITERAL(1, 29, 9), // "RunReader"
QT_MOC_LITERAL(2, 39, 0), // ""
QT_MOC_LITERAL(3, 40, 15), // "ConfigureSensor"
QT_MOC_LITERAL(4, 56, 23), // "witmotion_config_packet"
QT_MOC_LITERAL(5, 80, 13), // "config_packet"
QT_MOC_LITERAL(6, 94, 6), // "Packet"
QT_MOC_LITERAL(7, 101, 20), // "witmotion_datapacket"
QT_MOC_LITERAL(8, 122, 6), // "packet"
QT_MOC_LITERAL(9, 129, 5), // "Error"
QT_MOC_LITERAL(10, 135, 11) // "description"

    },
    "ROSWitmotionSensorController\0RunReader\0"
    "\0ConfigureSensor\0witmotion_config_packet\0"
    "config_packet\0Packet\0witmotion_datapacket\0"
    "packet\0Error\0description"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ROSWitmotionSensorController[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   34,    2, 0x06 /* Public */,
       3,    1,   35,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       6,    1,   38,    2, 0x0a /* Public */,
       9,    1,   41,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 4,    5,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 7,    8,
    QMetaType::Void, QMetaType::QString,   10,

       0        // eod
};

void ROSWitmotionSensorController::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<ROSWitmotionSensorController *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->RunReader(); break;
        case 1: _t->ConfigureSensor((*reinterpret_cast< const witmotion_config_packet(*)>(_a[1]))); break;
        case 2: _t->Packet((*reinterpret_cast< const witmotion_datapacket(*)>(_a[1]))); break;
        case 3: _t->Error((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (ROSWitmotionSensorController::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROSWitmotionSensorController::RunReader)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (ROSWitmotionSensorController::*)(const witmotion_config_packet & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROSWitmotionSensorController::ConfigureSensor)) {
                *result = 1;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject ROSWitmotionSensorController::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_ROSWitmotionSensorController.data,
    qt_meta_data_ROSWitmotionSensorController,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *ROSWitmotionSensorController::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ROSWitmotionSensorController::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_ROSWitmotionSensorController.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int ROSWitmotionSensorController::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 4;
    }
    return _id;
}

// SIGNAL 0
void ROSWitmotionSensorController::RunReader()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void ROSWitmotionSensorController::ConfigureSensor(const witmotion_config_packet & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
