// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "settings_export.hpp"
namespace rt950 {
enum class ImportPhase : uint8_t { Off,Waiting,Receiving,Checked,Applied,Cancelled,Timeout,Error };
// Address-free import of the canonical RT950CFG v1 container. The receiver
// acknowledges the header and each 32 payload bytes so LCD redraw cannot
// overflow the 64-byte IRQ ring. No flash API is available to this service.
template<class Port> class SettingsImport final {
public:
    bool active() const {return phase_==ImportPhase::Waiting || phase_==ImportPhase::Receiving;}
    ImportPhase phase() const {return phase_;}
    uint32_t received() const {return header_have_+payload_have_;}
    uint32_t total() const {return 48u+payload_size_;}
    unsigned channels() const {return channels_;}
    const char *version() const {return version_;}
    __attribute__((noinline)) void begin(uint32_t now) {
        cancel();header_have_=payload_have_=matched_=chunk_=0;payload_size_=0;schema_=0;channels_=0;
        version_[0]=0;step_=Step::Request;phase_=ImportPhase::Waiting;
        started_=progress_=now;port_.open();
    }
    void cancel() {
        if(active())port_.close();
        if(phase_!=ImportPhase::Off && phase_!=ImportPhase::Applied)phase_=ImportPhase::Cancelled;
    }
    __attribute__((noipa)) bool apply(Application &app,uint32_t now) {
        if(phase_!=ImportPhase::Checked || app.page()!=Page::Import || !app.import_confirm())return false;
        if(!app.import_snapshot(payload_,payload_size_,now,schema_)){phase_=ImportPhase::Error;return false;}
        phase_=ImportPhase::Applied;return true;
    }
    __attribute__((noinline)) void poll(uint32_t now,bool allowed) {
        if(!allowed){cancel();return;}
        if(!active())return;
        if(uint32_t(now-started_)>=60000u ||
           uint32_t(now-progress_)>=(phase_==ImportPhase::Waiting?30000u:2000u)) {
            port_.close();phase_=ImportPhase::Timeout;return;
        }
        if(step_==Step::Ack) {
            for(unsigned i=0;i<16 && ack_[ack_at_] && port_.ready();++i) {
                port_.put(uint8_t(ack_[ack_at_++]));progress_=now;
            }
            if(!ack_[ack_at_] && port_.done()) {
                if(terminal_!=ImportPhase::Off){port_.close();phase_=terminal_;}
                else step_=next_;
            }
            return;
        }
        if(step_==Step::Request && uint32_t(now-progress_)>=250u)matched_=0;
        uint8_t byte;
        for(unsigned i=0;i<32 && port_.read(byte);++i) {
            progress_=now;
            if(step_==Step::Request) {
                constexpr char request[]="RT950PUT\n";
                matched_=byte==uint8_t(request[matched_])?uint8_t(matched_+1):uint8_t(byte=='R');
                if(matched_==sizeof(request)-1) {
                    phase_=ImportPhase::Receiving;queue("READY\n",Step::Header);break;
                }
            } else if(step_==Step::Header) {
                header_[header_have_++]=byte;
                if(header_have_==sizeof header_) {
                    if(!valid_header())queue("ERROR\n",Step::Request,ImportPhase::Error);
                    else queue("NEXT\n",Step::Payload);
                    break;
                }
            } else if(step_==Step::Payload) {
                payload_[payload_have_++]=byte;++chunk_;
                if(payload_have_==payload_size_) {
                    if(validate())queue("CHECKED\n",Step::Request,ImportPhase::Checked);
                    else queue("ERROR\n",Step::Request,ImportPhase::Error);
                    break;
                }
                if(chunk_==32){chunk_=0;queue("NEXT\n",Step::Payload);break;}
            }
        }
    }
    Port &port(){return port_;}
private:
    enum class Step : uint8_t {Request,Header,Payload,Ack};
    void queue(const char *ack,Step next,ImportPhase terminal=ImportPhase::Off) {
        ack_=ack;ack_at_=0;next_=next;terminal_=terminal;step_=Step::Ack;
    }
    bool valid_header() {
        constexpr char magic[]="RT950CFG";
        for(unsigned i=0;i<8;++i)if(header_[i]!=uint8_t(magic[i]))return false;
        payload_size_=uint16_t(storage::word(header_+12));schema_=uint8_t(storage::word(header_+16));
        if(storage::word(header_+8)!=1 || !((storage::word(header_+12)==7992 && storage::word(header_+16)==1) ||
           (storage::word(header_+12)==Application::PortableSize && storage::word(header_+16)==Application::PortableSchema)) || storage::word(header_+40) ||
           storage::word(header_+44)!=~storage::crc_bytes(0xffffffffu,header_,44))return false;
        bool dot=false,ended=false,digit_before=false,digit_after=false;
        for(unsigned i=0;i<16;++i) {
            const uint8_t c=header_[24+i];version_[i]=char(c);
            if(!c){ended=true;continue;}
            if(ended)return false;
            if(c=='.' && !dot && digit_before){dot=true;continue;}
            if(c<'0' || c>'9')return false;
            if(dot)digit_after=true;else digit_before=true;
        }
        return ended && dot && digit_after;
    }
    bool validate() {
        if(storage::word(payload_)>1u ||
           storage::word(header_+20)!=~storage::crc_bytes(0xffffffffu,payload_,payload_size_))return false;
        Application candidate(false);
        if(!candidate.restore_portable_snapshot(payload_,payload_size_,schema_) || !candidate.compact_snapshot_size())return false;
        channels_=0;for(unsigned i=0;i<Application::MemoryCount;++i)channels_+=candidate.memory(i)->occupied;
        return true;
    }
    Port port_{};
    // v1 is immutable; future schemas must use a different decoder/size.
    uint8_t header_[48]{},payload_[Application::PortableSize]{};
    char version_[16]{};
    uint32_t started_=0,progress_=0;
    uint16_t header_have_=0,payload_have_=0,payload_size_=0;
    uint8_t schema_=0;
    unsigned channels_=0;
    const char *ack_=nullptr; // Keep the large static object in zero-initialized BSS.
    uint8_t matched_=0,chunk_=0,ack_at_=0;
    Step step_=Step::Request,next_=Step::Request;
    ImportPhase phase_=ImportPhase::Off,terminal_=ImportPhase::Off;
};
}
