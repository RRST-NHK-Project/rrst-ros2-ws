/****************************************************************************
** Meta object code from reading C++ file 'types.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.13)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../../witmotion_ros/witmotion-uart-qt/include/witmotion/types.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'types.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.13. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_witmotion__QAbstractWitmotionSensorReader_t {
    QByteArrayData data[11];
    char stringdata0[150];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_witmotion__QAbstractWitmotionSensorReader_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_witmotion__QAbstractWitmotionSensorReader_t qt_meta_stringdata_witmotion__QAbstractWitmotionSensorReader = {
    {
QT_MOC_LITERAL(0, 0, 41), // "witmotion::QAbstractWitmotion..."
QT_MOC_LITERAL(1, 42, 8), // "Acquired"
QT_MOC_LITERAL(2, 51, 0), // ""
QT_MOC_LITERAL(3, 52, 20), // "witmotion_datapacket"
QT_MOC_LITERAL(4, 73, 6), // "packet"
QT_MOC_LITERAL(5, 80, 5), // "Error"
QT_MOC_LITERAL(6, 86, 11), // "description"
QT_MOC_LITERAL(7, 98, 8), // "ReadData"
QT_MOC_LITERAL(8, 107, 10), // "SendConfig"
QT_MOC_LITERAL(9, 118, 23), // "witmotion_config_packet"
QT_MOC_LITERAL(10, 142, 7) // "RunPoll"

    },
    "witmotion::QAbstractWitmotionSensorReader\0"
    "Acquired\0\0witmotion_datapacket\0packet\0"
    "Error\0description\0ReadData\0SendConfig\0"
    "witmotion_config_packet\0RunPoll"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_witmotion__QAbstractWitmotionSensorReader[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   39,    2, 0x06 /* Public */,
       5,    1,   42,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       7,    0,   45,    2, 0x09 /* Protected */,
       8,    1,   46,    2, 0x0a /* Public */,
      10,    0,   49,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, QMetaType::QString,    6,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 9,    4,
    QMetaType::Void,

       0        // eod
};

void witmotion::QAbstractWitmotionSensorReader::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<QAbstractWitmotionSensorReader *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->Acquired((*reinterpret_cast< const witmotion_datapacket(*)>(_a[1]))); break;
        case 1: _t->Error((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 2: _t->ReadData(); break;
        case 3: _t->SendConfig((*reinterpret_cast< const witmotion_config_packet(*)>(_a[1]))); break;
        case 4: _t->RunPoll(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (QAbstractWitmotionSensorReader::*)(const witmotion_datapacket & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QAbstractWitmotionSensorReader::Acquired)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (QAbstractWitmotionSensorReader::*)(const QString & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QAbstractWitmotionSensorReader::Error)) {
                *result = 1;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject witmotion::QAbstractWitmotionSensorReader::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_witmotion__QAbstractWitmotionSensorReader.data,
    qt_meta_data_witmotion__QAbstractWitmotionSensorReader,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *witmotion::QAbstractWitmotionSensorReader::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *witmotion::QAbstractWitmotionSensorReader::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_witmotion__QAbstractWitmotionSensorReader.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int witmotion::QAbstractWitmotionSensorReader::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 5)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 5;
    }
    return _id;
}

// SIGNAL 0
void witmotion::QAbstractWitmotionSensorReader::Acquired(const witmotion_datapacket & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void witmotion::QAbstractWitmotionSensorReader::Error(const QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
