// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "aprs_rx.hpp"

namespace rt950::aprs {
class RecentMessages final {
public:
    static constexpr unsigned Capacity=3;
    struct Entry {
        char source[7]{},destination[10]{},body[68]{},id[6]{};
        uint8_t ssid=0;
        bool unread=false;
    };
    unsigned count(bool personal=false) const {return list(personal).size;}
    const Entry *at(unsigned index,bool personal=false) const {
        const auto &messages=list(personal);
        return index<messages.size?&messages.entries[index]:nullptr;
    }
    bool personal_unread() const {
        for(unsigned i=0;i<personal_.size;++i)
            if(personal_.entries[i].unread)return true;
        return false;
    }
    void read_personal(unsigned index) {
        if(index<personal_.size)personal_.entries[index].unread=false;
    }
    void clear() {all_.size=0;personal_.size=0;}
    void select_identity(const char *callsign,uint8_t ssid) {
        char address[10]{};
        unsigned length=0;
        while(length<6 && callsign[length]) {
            address[length]=callsign[length];++length;
        }
        if(length && ssid) {
            address[length++]='-';
            if(ssid>=10)address[length++]='1';
            address[length]=char('0'+ssid%10);
        }
        if(equal(address,address_))return;
        for(unsigned i=0;i<sizeof address;++i)address_[i]=address[i];
        personal_.size=0;
    }

    void record(const uint8_t *frame,size_t size) {
        size_t at=0;
        if(!ui_payload(frame,size,at) || size-at<11 || frame[at]!=':' || frame[at+10]!=':')return;
        Entry entry;
        unsigned address_size=9;
        while(address_size && frame[at+address_size]==' ')--address_size;
        if(!address_size)return;
        for(unsigned i=0;i<address_size;++i) {
            const uint8_t ch=frame[at+1+i];
            if(ch<' ' || ch>'~' || ch==':')return;
            entry.destination[i]=char(ch);
        }
        at+=11;
        unsigned length=0;
        while(at<size && frame[at]!='{') {
            const uint8_t ch=frame[at++];
            if(length==67 || ch<' ' || ch>'~' || ch=='|' || ch=='~')return;
            entry.body[length++]=char(ch);
        }
        if(!length)return;
        const char *body=entry.body;
        if(at==size && length>3 && ((body[0]=='a' && body[1]=='c' && body[2]=='k') ||
                                   (body[0]=='r' && body[1]=='e' && body[2]=='j')) &&
           message_id(body+3,length-3))return;
        if(at<size) {
            const size_t id_start=++at;
            while(at<size && frame[at]!='}')++at;
            const size_t id_size=at-id_start;
            if(!message_id(reinterpret_cast<const char *>(frame+id_start),id_size))return;
            for(unsigned i=0;i<id_size;++i)entry.id[i]=char(frame[id_start+i]);
            if(at<size && (id_size!=2 ||
                !reply_id(reinterpret_cast<const char *>(frame+at+1),size-at-1)))return;
        }
        for(unsigned i=0;i<6 && (frame[7+i]>>1)!=' ';++i)
            entry.source[i]=char(frame[7+i]>>1);
        entry.ssid=(frame[13]>>1)&15u;

        all_.push(entry);
        if(address_[0] && addressed_to_me(entry.destination)) {
            entry.unread=true;
            personal_.push(entry);
        }
    }
private:
    static bool alphanumeric(char ch) {
        return (ch>='0' && ch<='9') || (ch>='A' && ch<='Z') || (ch>='a' && ch<='z');
    }
    // APRS 1.1 Reply-ACK appends '}' and optionally the two-character ACK.
    static bool reply_id(const char *id,size_t size) {
        return !size || (size==2 && alphanumeric(id[0]) && alphanumeric(id[1]));
    }
    static bool message_id(const char *id,size_t size) {
        if(!size || size>5)return false;
        for(size_t i=0;i<size;++i) {
            if(!alphanumeric(id[i]))
                return i==2 && id[i]=='}' && reply_id(id+i+1,size-i-1);
        }
        return true;
    }
    static bool equal(const char *a,const char *b) {
        while(*a && *a==*b){++a;++b;}
        return *a==*b;
    }
    static char uppercase(char ch) {return ch>='a' && ch<='z'?char(ch-'a'+'A'):ch;}
    bool addressed_to_me(const char *destination) const {
        unsigned i=0;
        bool explicit_ssid=false;
        while(address_[i] && uppercase(destination[i])==uppercase(address_[i])) {
            explicit_ssid|=address_[i]=='-';
            ++i;
        }
        return !address_[i] && (!destination[i] ||
            (!explicit_ssid && destination[i]=='-' && destination[i+1]=='0' && !destination[i+2]));
    }
    struct List {
        Entry entries[Capacity]{};
        unsigned size=0;
        void push(const Entry &entry) {
            for(unsigned i=0;i<size;++i) {
                const auto &old=entries[i];
                if(old.ssid==entry.ssid && equal(old.source,entry.source) &&
                   equal(old.destination,entry.destination) && equal(old.id,entry.id) &&
                   equal(old.body,entry.body))return;
            }
            if(size<Capacity)++size;
            for(unsigned i=size-1;i>0;--i)entries[i]=entries[i-1];
            entries[0]=entry;
        }
    };
    const List &list(bool personal) const {return personal?personal_:all_;}
    List all_{},personal_{};
    char address_[10]{};
};
}
