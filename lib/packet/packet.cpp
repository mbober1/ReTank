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
    this->left = (int8_t)(data[1]);
    this->right = (int8_t)(data[3]);
}

char EnginePacket::getType() {
    return 'E';
}

std::string EnginePacket::prepare() {
    std::string tmp;
    tmp += this->getType();
    tmp += this->left;
    tmp += ' ';
    tmp += this->right;
    tmp += ';';
    tmp += Packet::checksum(tmp);
    return tmp;
}