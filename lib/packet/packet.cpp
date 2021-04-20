#include "packet.hpp"

Packet::Packet() {}
Packet::~Packet() {}

uint8_t Packet::checksum(const std::string &data) {
    return CRC::Calculate(data.c_str(), data.size(), CRC::CRC_8());
}

Packet* Packet::decode(std::string &data) {
    uint8_t crc = data.at(data.size() - 1);
    data.pop_back();

    if(Packet::checksum(data) == crc) {
        char type = data.at(0);
        switch (type)
        {
        case 'P':
            return new PingPacket();

        case 'C':
            return new ClosePacket();

        case 'E':
            return new EnginePacket(data);

        case 'B':
            return new BatteryPacket(data);

        case 'D':
            return new DistancePacket(data);

        case 'G':
            return new GyroPacket(data);

        case 'A':
            return new AcceloPacket(data);

        case 'S':
            return new SpeedPacket(data);

        default:
            return nullptr;
            printf("unknown packet\n");
            break;
        }
    } else {
        printf("bad crc :(\n");
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
    tmp += Packet::checksum(tmp);
    tmp += ';';
    return tmp;
}






ClosePacket::ClosePacket() {}
ClosePacket::~ClosePacket() {}

char ClosePacket::getType() {
    return 'C';
}

std::string ClosePacket::prepare() {
    std::string tmp;
    tmp += this->getType();
    tmp += Packet::checksum(tmp);
    tmp += ';';
    return tmp;
}





EnginePacket::EnginePacket(const int8_t &left, const int8_t &right) : left(left), right(right) {}
EnginePacket::EnginePacket() {}
EnginePacket::~EnginePacket() {}

EnginePacket::EnginePacket(const std::string &data) {
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
    tmp += Packet::checksum(tmp);
    tmp += ';';
    return tmp;
}



BatteryPacket::BatteryPacket(const uint8_t &level) : level(level) {}
BatteryPacket::~BatteryPacket() {}

BatteryPacket::BatteryPacket(const std::string &data) {
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
    tmp += Packet::checksum(tmp);
    tmp += ';';
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
    tmp += Packet::checksum(tmp);
    tmp += ';';
    return tmp;
}





GyroPacket::GyroPacket(const int8_t &x, const int8_t &y, const int8_t &z) : x(x), y(y), z(z) {}
GyroPacket::~GyroPacket() {}

GyroPacket::GyroPacket(std::string data) {
    if(!data.empty()) {
        data.erase(0, 1);

        int separator = data.find(' ') + 1;
        std::string parse = data.substr(0, separator);
        data.erase(0, separator);
        this->x = std::atoi(parse.c_str());

        separator = data.find(' ') + 1;
        parse = data.substr(0, separator);
        data.erase(0, separator);
        this->y = std::atoi(parse.c_str());

        separator = data.find(';') + 1;
        parse = data.substr(0, separator);
        this->z = std::atoi(parse.c_str());
    }
}

char GyroPacket::getType() {
    return 'G';
}

std::string GyroPacket::prepare() {
    std::string tmp;
    tmp += this->getType();
    tmp += std::to_string(this->x);
    tmp += ' ';
    tmp += std::to_string(this->y);
    tmp += ' ';
    tmp += std::to_string(this->z);
    tmp += Packet::checksum(tmp);
    tmp += ';';
    return tmp;
}




AcceloPacket::AcceloPacket(const int8_t &x, const int8_t &y, const int8_t &z) : x(x), y(y), z(z) {}
AcceloPacket::~AcceloPacket() {}

AcceloPacket::AcceloPacket(std::string data) {
    if(!data.empty()) {
        data.erase(0, 1);

        int separator = data.find(' ') + 1;
        std::string parse = data.substr(0, separator);
        data.erase(0, separator);
        this->x = std::atoi(parse.c_str());

        separator = data.find(' ') + 1;
        parse = data.substr(0, separator);
        data.erase(0, separator);
        this->y = std::atoi(parse.c_str());

        separator = data.find(';') + 1;
        parse = data.substr(0, separator);
        this->z = std::atoi(parse.c_str());
    }
}

char AcceloPacket::getType() {
    return 'A';
}

std::string AcceloPacket::prepare() {
    std::string tmp;
    tmp += this->getType();
    tmp += std::to_string(this->x);
    tmp += ' ';
    tmp += std::to_string(this->y);
    tmp += ' ';
    tmp += std::to_string(this->z);
    tmp += Packet::checksum(tmp);
    tmp += ';';
    return tmp;
}





SpeedPacket::SpeedPacket(const int8_t &left, const int8_t &right) : left(left), right(right) {}
SpeedPacket::~SpeedPacket() {}

SpeedPacket::SpeedPacket(const std::string &data) {
    if(!data.empty()) {
        int separator = data.find(' ');
        std::string parse = data.substr(1, separator - 1);
        this->left = std::atoi(parse.c_str());
        parse = data.substr(separator + 1, data.find(';') - separator - 1);
        this->right = std::atoi(parse.c_str());
    }
}

char SpeedPacket::getType() {
    return 'S';
}

std::string SpeedPacket::prepare() {
    std::string tmp;
    tmp += this->getType();
    tmp += std::to_string(this->left);
    tmp += ' ';
    tmp += std::to_string(this->right);
    tmp += Packet::checksum(tmp);
    tmp += ';';
    return tmp;
}