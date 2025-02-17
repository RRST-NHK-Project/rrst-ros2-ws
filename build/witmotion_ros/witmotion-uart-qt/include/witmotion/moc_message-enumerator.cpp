/****************************************************************************
** Meta object code from reading C++ file 'message-enumerator.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.13)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../../witmotion_ros/witmotion-uart-qt/include/witmotion/message-enumerator.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'message-enumerator.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.13. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_witmotion__QGeneralSensorController_t {
    QByteArrayData data[8];
    char stringdata0[100];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_witmotion__QGeneralSensorController_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_witmotion__QGeneralSensorController_t qt_meta_stringdata_witmotion__QGeneralSensorController = {
    {
QT_MOC_LITERAL(0, 0, 35), // "witmotion::QGeneralSensorCont..."
QT_MOC_LITERAL(1, 36, 9), // "RunReader"
QT_MOC_LITERAL(2, 46, 0), // ""
QT_MOC_LITERAL(3, 47, 6), // "Packet"
QT_MOC_LITERAL(4, 54, 20), // "witmotion_datapacket"
QT_MOC_LITERAL(5, 75, 6), // "packet"
QT_MOC_LITERAL(6, 82, 5), // "Error"
QT_MOC_LITERAL(7, 88, 11) // "description"

    },
    "witmotion::QGeneralSensorController\0"
    "RunReader\0\0Packet\0witmotion_datapacket\0"
    "packet\0Error\0description"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_witmotion__QGeneralSensorController[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   29,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       3,    1,   30,    2, 0x0a /* Public */,
       6,    1,   33,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 4,    5,
    QMetaType::Void, QMetaType::QString,    7,

       0        // eod
};

void witmotion::QGeneralSensorController::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<QGeneralSensorController *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->RunReader(); break;
        case 1: _t->Packet((*reinterpret_cast< const witmotion_datapacket(*)>(_a[1]))); break;
        case 2: _t->Error((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (QGeneralSensorController::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QGeneralSensorController::RunReader)) {
                *result = 0;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject witmotion::QGeneralSensorController::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_witmotion__QGeneralSensorController.data,
    qt_meta_data_witmotion__QGeneralSensorController,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *witmotion::QGeneralSensorController::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *witmotion::QGeneralSensorController::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_witmotion__QGeneralSensorController.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int witmotion::QGeneralSensorController::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 3)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 3;
    }
    return _id;
}

// SIGNAL 0
void witmotion::QGeneralSensorController::RunReader()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
