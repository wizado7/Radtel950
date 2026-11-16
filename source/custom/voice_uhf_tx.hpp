// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "voice_ptt.hpp"
#include "voice_tx_route.hpp"
#include "voice_tx_registers.hpp"

namespace rt950::voice {
// Hardware adapters must provide settled channel
// observations and exclusive ownership of the RF bus, route and DAC2.
template<class Board>
class UhfTransmitter final {
public:
    enum class Channel : uint8_t { Pending, Clear, Busy, Failed };
    enum class Fault : uint8_t { None, Receiver, Route, Sample, Chip, Values, Setup, Readback, Hold };
    Fault fault() const {return fault_;}
    uint8_t failed_register() const {return failed_register_;}
    explicit UhfTransmitter(Board &board):board_(board){}

    bool begin(const TxRequest &request) {
        fault_=Fault::None;failed_register_=0;
        if(active_ || !board_.acquire())return fail(Fault::Receiver);
        active_=true;
        board_.bias(0);
        frequency_=request.frequency;
        busy_lock_=request.busy_lock;
        clear_=false;
        return board_.listen(frequency_) || fail(Fault::Route);
    }
    Ptt::Preparation poll_ready(uint32_t now) {
        if(!active_)return Ptt::Preparation::Failed;
        const Channel channel=board_.channel(now);
        if(channel==Channel::Pending)return Ptt::Preparation::Pending;
        if(channel==Channel::Failed){fault_=Fault::Sample;return Ptt::Preparation::Failed;}
        clear_=channel==Channel::Clear;
        if(busy_lock_ && !clear_)return Ptt::Preparation::Busy;
        uint16_t mic=0;
        if(!board_.read(0x7d,mic) || mic==0xffff) {
            fault_=Fault::Chip;return Ptt::Preparation::Failed;
        }
        if(!board_.tx_values(mic,values_)) {
            fault_=Fault::Values;return Ptt::Preparation::Failed;
        }
        return Ptt::Preparation::Ready;
    }
    bool channel_clear() const {return clear_;}
    bool healthy() const {return active_ && board_.healthy();}
    bool key() {
        if(!active_ || !healthy() || !values_.pa_bias)return fail(Fault::Hold);
        board_.mute_receiver();
        // Prepare the route with zero PA bias. Delegate board-specific
        // modulation, clock and RF mode transitions to the board adapter.
        prepare_uhf_route(board_);
        if(!board_.configure_voice(values_) || !board_.finish_voice())
            return fail(Fault::Setup);
        if(!verify_voice_tx(board_,values_,board_.power_selector(),TxActiveWord,&failed_register_))
            return fail(Fault::Readback);
        if(cb_fm_frequency(frequency_)) {
            board_.set(TxRoutePin::Pe7,false);
            board_.set(TxRoutePin::Pb1,false);
            board_.set(TxRoutePin::Pe14,true);
        } else if(vhf_tx_frequency(frequency_)) {
            board_.set(TxRoutePin::Pe7,false);
            board_.set(TxRoutePin::Pb1,true);
            board_.set(TxRoutePin::Pe14,false);
        } else select_uhf_tx_route(board_);
        return raise_bias() || fail(Fault::Hold);
    }
    bool set_signal(const TxSignal &signal) {
        if(!active_ || !values_.pa_bias)return false;
        board_.bias(0);
        if(healthy() && board_.configure_signal(signal) &&
           verify_voice_tx(board_,values_,board_.power_selector(),signal?TxSignalWord:TxActiveWord) &&
           raise_bias())return true;
        stop();
        return false;
    }
    bool start_tail() {return active_ && board_.start_tail();}
    void stop() {
        if(!active_)return;
        board_.bias(0);
        board_.restore_rx();
        active_=false;clear_=false;values_={};
    }
private:
    bool fail(Fault reason){fault_=reason;return false;}
    bool raise_bias() {
        constexpr unsigned steps=4;
        for(unsigned step=1;step<=steps;++step) {
            if(!healthy()) {
                board_.bias(0);
                return false;
            }
            board_.bias(uint8_t(unsigned(values_.pa_bias)*step/steps));
            if(step<steps)board_.delay_us(1000);
        }
        if(healthy())return true;
        board_.bias(0);
        return false;
    }
    Board &board_;
    UhfTxValues values_{};
    uint32_t frequency_=0;
    Fault fault_=Fault::None;
    uint8_t failed_register_=0;
    bool active_=false,busy_lock_=true,clear_=false;
};
}
