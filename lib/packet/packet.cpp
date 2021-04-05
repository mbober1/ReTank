#include "packet.hpp"

Packet::Packet() {}
Packet::~Packet() {}

uint8_t Packet::checksum(const std::string &data) {
    return CRC::Calculate(data.c_str(), data.size(), CRC::CRC_8());
}

Packet* Packet::decode(std::string &data) {
    char crc = data.at(data.size() - 1);
    data.pop_back();

    if(Packet::checksum(data) == (uint8_t)crc) {
        char type = data.at(0);
        switch (type)
        {
        case 'P':
            return new PingPacket();

        case 'E':
            return new EnginePacket(data);

        case 'B':
            return new BatteryPacket(data);

        case 'D':
            return new DistancePacket(data);
        
        default:
            return nullptr;
            printf("unknown packet");
            break;
        }
    } else {
        printf("złe crc :(");
        return nullptr;
    }

}




PingPacket::PingPacket() {}
PingPacket::~PingPacket() {}

char PingPacket::getType() {
    return 'P';
}


std::string PingPacket::prepare() {
    std::string tmp;
    tmp += this->getType();
    tmp += ';';
    tmp += Packet::checksum(tmp);
    return tmp;
}




EnginePacket::EnginePacket(const int8_t &left, const int8_t &right) : left(left), right(right) {}
EnginePacket::~EnginePacket() {}

EnginePacket::EnginePacket(const std::string &data)
{
    if(!data.empty()) {
        int separator = data.find(' ');
        std::string parse = data.substr(1, separator - 1);
        this->left = std::atoi(parse.c_str());
        parse = data.substr(separator + 1, data.find(';') - separator - 1);
        this->right = std::atoi(parse.c_str());
    }
}

char EnginePacket::getType() {
    return 'E';
}

std::string EnginePacket::prepare() {
    std::string tmp;
    tmp += this->getType();
    tmp += std::to_string(this->left);
    tmp += ' ';
    tmp += std::to_string(this->right);
    tmp += ';';
    tmp += Packet::checksum(tmp);
    return tmp;
}



BatteryPacket::BatteryPacket(const uint8_t &level) : level(level) {}
BatteryPacket::~BatteryPacket() {}

BatteryPacket::BatteryPacket(const std::string &data)
{
    if(!data.empty()) {
        int separator = data.find(';');
        std::string parse = data.substr(1, separator - 1);
        this->level = std::atoi(parse.c_str());
    }
}

char BatteryPacket::getType() {
    return 'B';
}

std::string BatteryPacket::prepare() {
    std::string tmp;
    tmp += this->getType();
    tmp += std::to_string(this->level);
    tmp += ';';
    tmp += Packet::checksum(tmp);
    return tmp;
}





DistancePacket::DistancePacket(const uint8_t &distance) : distance(distance) {}
DistancePacket::~DistancePacket() {}

DistancePacket::DistancePacket(const std::string &data) {
    if(!data.empty()) {
        int separator = data.find(';');
        std::string parse = data.substr(1, separator - 1);
        this->distance = std::atoi(parse.c_str());
    }
}

char DistancePacket::getType() {
    return 'D';
}

std::string DistancePacket::prepare() {
    std::string tmp;
    tmp += this->getType();
    tmp += std::to_string(this->distance);
    tmp += ';';
    tmp += Packet::checksum(tmp);
    return tmp;
}