/****************************************************************************
** Meta object code from reading C++ file 'serial.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.13)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../../witmotion_ros/witmotion-uart-qt/include/witmotion/serial.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'serial.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.13. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_witmotion__QBaseSerialWitmotionSensorReader_t {
    QByteArrayData data[1];
    char stringdata0[44];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_witmotion__QBaseSerialWitmotionSensorReader_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_witmotion__QBaseSerialWitmotionSensorReader_t qt_meta_stringdata_witmotion__QBaseSerialWitmotionSensorReader = {
    {
QT_MOC_LITERAL(0, 0, 43) // "witmotion::QBaseSerialWitmoti..."

    },
    "witmotion::QBaseSerialWitmotionSensorReader"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_witmotion__QBaseSerialWitmotionSensorReader[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

       0        // eod
};

void witmotion::QBaseSerialWitmotionSensorReader::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    (void)_o;
    (void)_id;
    (void)_c;
    (void)_a;
}

QT_INIT_METAOBJECT const QMetaObject witmotion::QBaseSerialWitmotionSensorReader::staticMetaObject = { {
    QMetaObject::SuperData::link<QAbstractWitmotionSensorReader::staticMetaObject>(),
    qt_meta_stringdata_witmotion__QBaseSerialWitmotionSensorReader.data,
    qt_meta_data_witmotion__QBaseSerialWitmotionSensorReader,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *witmotion::QBaseSerialWitmotionSensorReader::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *witmotion::QBaseSerialWitmotionSensorReader::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_witmotion__QBaseSerialWitmotionSensorReader.stringdata0))
        return static_cast<void*>(this);
    return QAbstractWitmotionSensorReader::qt_metacast(_clname);
}

int witmotion::QBaseSerialWitmotionSensorReader::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QAbstractWitmotionSensorReader::qt_metacall(_c, _id, _a);
    return _id;
}
struct qt_meta_stringdata_witmotion__QAbstractWitmotionSensorController_t {
    QByteArrayData data[12];
    char stringdata0[168];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_witmotion__QAbstractWitmotionSensorController_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_witmotion__QAbstractWitmotionSensorController_t qt_meta_stringdata_witmotion__QAbstractWitmotionSensorController = {
    {
QT_MOC_LITERAL(0, 0, 45), // "witmotion::QAbstractWitmotion..."
QT_MOC_LITERAL(1, 46, 9), // "RunReader"
QT_MOC_LITERAL(2, 56, 0), // ""
QT_MOC_LITERAL(3, 57, 13), // "ErrorOccurred"
QT_MOC_LITERAL(4, 71, 11), // "description"
QT_MOC_LITERAL(5, 83, 8), // "Acquired"
QT_MOC_LITERAL(6, 92, 20), // "witmotion_datapacket"
QT_MOC_LITERAL(7, 113, 6), // "packet"
QT_MOC_LITERAL(8, 120, 10), // "SendConfig"
QT_MOC_LITERAL(9, 131, 23), // "witmotion_config_packet"
QT_MOC_LITERAL(10, 155, 6), // "Packet"
QT_MOC_LITERAL(11, 162, 5) // "Error"

    },
    "witmotion::QAbstractWitmotionSensorController\0"
    "RunReader\0\0ErrorOccurred\0description\0"
    "Acquired\0witmotion_datapacket\0packet\0"
    "SendConfig\0witmotion_config_packet\0"
    "Packet\0Error"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_witmotion__QAbstractWitmotionSensorController[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   44,    2, 0x06 /* Public */,
       3,    1,   45,    2, 0x06 /* Public */,
       5,    1,   48,    2, 0x06 /* Public */,
       8,    1,   51,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      10,    1,   54,    2, 0x0a /* Public */,
      11,    1,   57,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,    4,
    QMetaType::Void, 0x80000000 | 6,    7,
    QMetaType::Void, 0x80000000 | 9,    7,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 6,    7,
    QMetaType::Void, QMetaType::QString,    4,

       0        // eod
};

void witmotion::QAbstractWitmotionSensorController::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<QAbstractWitmotionSensorController *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->RunReader(); break;
        case 1: _t->ErrorOccurred((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 2: _t->Acquired((*reinterpret_cast< const witmotion_datapacket(*)>(_a[1]))); break;
        case 3: _t->SendConfig((*reinterpret_cast< const witmotion_config_packet(*)>(_a[1]))); break;
        case 4: _t->Packet((*reinterpret_cast< const witmotion_datapacket(*)>(_a[1]))); break;
        case 5: _t->Error((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (QAbstractWitmotionSensorController::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QAbstractWitmotionSensorController::RunReader)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (QAbstractWitmotionSensorController::*)(const QString & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QAbstractWitmotionSensorController::ErrorOccurred)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (QAbstractWitmotionSensorController::*)(const witmotion_datapacket & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QAbstractWitmotionSensorController::Acquired)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (QAbstractWitmotionSensorController::*)(const witmotion_config_packet & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QAbstractWitmotionSensorController::SendConfig)) {
                *result = 3;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject witmotion::QAbstractWitmotionSensorController::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_witmotion__QAbstractWitmotionSensorController.data,
    qt_meta_data_witmotion__QAbstractWitmotionSensorController,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *witmotion::QAbstractWitmotionSensorController::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *witmotion::QAbstractWitmotionSensorController::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_witmotion__QAbstractWitmotionSensorController.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int witmotion::QAbstractWitmotionSensorController::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
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

// SIGNAL 0
void witmotion::QAbstractWitmotionSensorController::RunReader()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void witmotion::QAbstractWitmotionSensorController::ErrorOccurred(const QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void witmotion::QAbstractWitmotionSensorController::Acquired(const witmotion_datapacket & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void witmotion::QAbstractWitmotionSensorController::SendConfig(const witmotion_config_packet & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
