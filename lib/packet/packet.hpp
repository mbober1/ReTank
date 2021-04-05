#pragma once
#include <string>
#include "crc.h"

class Packet {
public:
    Packet();
    Packet(const std::string &data);
    virtual ~Packet() = 0;
    virtual std::string prepare() = 0;
    static Packet* decode(std::string &data);
    virtual char getType() = 0;
    static uint8_t checksum(const std::string &data);
};


class PingPacket : public Packet {
public:
    PingPacket();
    ~PingPacket();
    virtual std::string prepare();
    virtual char getType();
};


class EnginePacket : public Packet {
public:
    EnginePacket(const std::string &data);
    EnginePacket(const int8_t &left, const int8_t &right);
    ~EnginePacket();
    virtual std::string prepare();
    int left;
    int right;
    virtual char getType();
};


class BatteryPacket : public Packet {
public:
    BatteryPacket(const std::string &data);
    BatteryPacket(const uint8_t &level);
    ~BatteryPacket();
    virtual std::string prepare();
    int level;
    virtual char getType();
};


class DistancePacket : public Packet {
public:
    DistancePacket(const std::string &data);
    DistancePacket(const uint8_t &distance);
    ~DistancePacket();
    virtual std::string prepare();
    int distance;
    virtual char getType();
};