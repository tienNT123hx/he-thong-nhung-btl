#ifndef I2C_BUS_H
#define I2C_BUS_H

#include <systemc>
#include "data_types.h"

using namespace sc_core;

SC_MODULE(I2CBus) {
    sc_fifo<AccelData> fifo;

    SC_CTOR(I2CBus) : fifo(16) {}
};

#endif
