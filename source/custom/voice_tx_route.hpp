// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

namespace rt950::voice {
// GPIO routing for OEM band path 1, 0x0801D5A4. This module is
// independent of the radio backend; PA bias and RF mode are separate.
enum class TxRoutePin { Pe7,Pe12,Pe13,Pe14,Pb0,Pb1 };

template<class Pins>
void prepare_uhf_route(Pins &pins) {
    pins.set(TxRoutePin::Pe13,false);
    pins.set(TxRoutePin::Pe7,false);
    pins.set(TxRoutePin::Pb1,false);
    pins.set(TxRoutePin::Pe12,false);
    pins.set(TxRoutePin::Pb0,false);
    pins.set(TxRoutePin::Pe14,false);
}

// Call only after the transmitter has been prepared with verified
// calibration. The stock wrapper calls this after 0x0801F2A8.
template<class Pins>
void select_uhf_tx_route(Pins &pins) {
    pins.set(TxRoutePin::Pb1,false);
    pins.set(TxRoutePin::Pe7,true);
    pins.set(TxRoutePin::Pe14,false);
}
}
