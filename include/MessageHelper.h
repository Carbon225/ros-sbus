#ifndef SRC_MESSAGEHELPER_H
#define SRC_MESSAGEHELPER_H

#include "ros_sbus/SbusPacket.h"
#include "SBUS.h"

#define SbusConvertPacketMessage(from, to)        \
{                                                 \
    for (int _i = 0; _i < 16; _i++)               \
    {                                             \
        (to).channels[_i] = (from).channels[_i];  \
    }                                             \
    (to).ch17 = (from).ch17;                      \
    (to).ch18 = (from).ch18;                      \
    (to).failsafe = (from).failsafe;              \
    (to).frameLost = (from).frameLost;            \
}

#endif //SRC_MESSAGEHELPER_H
