// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "application.hpp"
#include "storage_record.hpp"
#include "firmware_version.hpp"
namespace rt950 {
enum class BackupPhase : uint8_t { Off,Waiting,Sending,Complete,Cancelled,Timeout,Error };
// Read-only snapshot service. The only accepted command is RT950CFG + LF.
// Header: magic[8], protocol, payload length, snapshot schema, payload CRC,
// firmware[16], reserved flags=0, header CRC. Payload is canonical LE words.
template<class Port> class SettingsExport final {
public:
    static constexpr size_t HeaderSize=48;
    bool active() const {return phase_==BackupPhase::Waiting || phase_==BackupPhase::Sending;}
    BackupPhase phase() const {return phase_;}
    uint32_t sent() const {return sent_;}
    uint32_t total() const {return uint32_t(HeaderSize+size_);}
    __attribute__((noinline)) void begin(const Application &app,uint32_t now) {
        cancel();size_=app.save_portable_snapshot(payload_,sizeof payload_);
        sent_=matched_=0;
        if(!size_){phase_=BackupPhase::Error;return;}
        for(auto &byte:header_)byte=0;
        constexpr char magic[]="RT950CFG";
        for(unsigned i=0;i<8;++i)header_[i]=uint8_t(magic[i]);
        storage::put(header_+8,1);storage::put(header_+12,uint32_t(size_));
        storage::put(header_+16,Application::PortableSchema);
        storage::put(header_+20,~storage::crc_bytes(0xffffffffu,payload_,size_));
        constexpr char version[]=RT950_CUSTOM_VERSION;
        static_assert(sizeof version<=16);
        for(unsigned i=0;i<sizeof version;++i)header_[24+i]=uint8_t(version[i]);
        storage::put(header_+44,~storage::crc_bytes(0xffffffffu,header_,44));
        port_.open();phase_=BackupPhase::Waiting;started_=progressed_=received_=now;
    }
    void cancel() {if(active())port_.close();if(active())phase_=BackupPhase::Cancelled;matched_=0;}
    void poll(uint32_t now,bool allowed) {
        if(!active())return;
        if(!allowed){cancel();return;}
        if(uint32_t(now-started_)>=(phase_==BackupPhase::Waiting?30000u:15000u) ||
           (phase_==BackupPhase::Sending && uint32_t(now-progressed_)>=500u)) {
            port_.close();phase_=BackupPhase::Timeout;return;
        }
        if(uint32_t(now-received_)>=250u)matched_=0;
        uint8_t byte=0;
        for(unsigned i=0;i<32 && port_.read(byte);++i) {
            received_=now;
            if(phase_!=BackupPhase::Waiting)continue;
            constexpr char request[]="RT950CFG\n";
            matched_=byte==uint8_t(request[matched_])?uint8_t(matched_+1):uint8_t(byte=='R'?1:0);
            if(matched_==sizeof(request)-1){phase_=BackupPhase::Sending;started_=progressed_=now;matched_=0;}
        }
        if(phase_!=BackupPhase::Sending)return;
        for(unsigned i=0;i<32 && sent_<total() && port_.ready();++i) {
            port_.put(sent_<HeaderSize?header_[sent_]:payload_[sent_-HeaderSize]);
            ++sent_;progressed_=now;
        }
        if(sent_==total() && port_.done()){port_.close();phase_=BackupPhase::Complete;}
    }
    Port &port(){return port_;}
private:
    Port port_{};
    uint8_t payload_[Application::SnapshotCapacity]{},header_[HeaderSize]{};
    size_t size_=0;
    uint32_t sent_=0,started_=0,progressed_=0,received_=0;
    uint8_t matched_=0;
    BackupPhase phase_=BackupPhase::Off;
};
class BackupUart final {
public:
    void open();void close();bool read(uint8_t &);bool ready() const;bool done() const;void put(uint8_t);
private:
    uint32_t crh_=0,odr_=0,clock_=0;
    bool opened_=false;
};
}
