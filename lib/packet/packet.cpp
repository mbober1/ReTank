#include "packet.hpp"


/**
 * A constructor.
 */
Packet::Packet() {}


/**
 * A destructor.
 */
Packet::~Packet() {}


/**
 * Calculate CRC for given string
 * @param[in] data String with data frame.
 * @return Calculated crc.
 */
uint8_t Packet::checksum(const std::string &data) {
    return CRC::Calculate(data.c_str(), data.size(), CRC::CRC_8());
}


/**
 * Decode incoming packet.
 * @param[in] data String with data frame.
 * @return Pointer to decoded packet.
 */
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



/**
 * A constructor.
 */
PingPacket::PingPacket() {}

/**
 * A destructor.
 */
PingPacket::~PingPacket() {}

/**
 * Get type of packet.
 * @return Type.
 */
char PingPacket::getType() {
    return 'P';
}


/**
 * Prepare packet for sending.
 * @return String of data.
 */
std::string PingPacket::prepare() {
    std::string tmp;
    tmp += this->getType();
    tmp += Packet::checksum(tmp);
    tmp += ';';
    return tmp;
}






/**
 * A constructor.
 */
ClosePacket::ClosePacket() {}


/**
 * A destructor.
 */
ClosePacket::~ClosePacket() {}


/**
 * Get type of packet.
 * @return Type.
 */
char ClosePacket::getType() {
    return 'C';
}


/**
 * Prepare packet for sending.
 * @return String of data.
 */
std::string ClosePacket::prepare() {
    std::string tmp;
    tmp += this->getType();
    tmp += Packet::checksum(tmp);
    tmp += ';';
    return tmp;
}





/**
 * A constructor.
 * @param[in] left      Engine power from -100 to 100.
 * @param[in] right     Engine power from -100 to 100.
 */
EnginePacket::EnginePacket(const int8_t &left, const int8_t &right) : left(left), right(right) {}


/**
 * A constructor.
 */
EnginePacket::EnginePacket() {}


/**
 * A destructor.
 */
EnginePacket::~EnginePacket() {}


/**
 * A constructor for parsing data.
 * @param[in] data  Data string.
 */
EnginePacket::EnginePacket(const std::string &data) {
    if(!data.empty()) {
        int separator = data.find(' ');
        std::string parse = data.substr(1, separator - 1);
        this->left = std::atoi(parse.c_str());
        parse = data.substr(separator + 1, data.find(';') - separator - 1);
        this->right = std::atoi(parse.c_str());
    }
}


/**
 * Get type of packet.
 * @return Type.
 */
char EnginePacket::getType() {
    return 'E';
}


/**
 * Prepare packet for sending.
 * @return String of data.
 */
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






/**
 * A constructor.
 * @param[in] level Battery level 0-100.
 */
BatteryPacket::BatteryPacket(const uint8_t &level) : level(level) {}


/**
 * A destructor.
 */
BatteryPacket::~BatteryPacket() {}


/**
 * A constructor for parsing data.
 * @param[in] data Data string.
 */
BatteryPacket::BatteryPacket(const std::string &data) {
    if(!data.empty()) {
        int separator = data.find(';');
        std::string parse = data.substr(1, separator - 1);
        this->level = std::atoi(parse.c_str());
    }
}


/**
 * Get type of packet.
 * @return Type.
 */
char BatteryPacket::getType() {
    return 'B';
}


/**
 * Prepare packet for sending.
 * @return String of data.
 */
std::string BatteryPacket::prepare() {
    std::string tmp;
    tmp += this->getType();
    tmp += std::to_string(this->level);
    tmp += Packet::checksum(tmp);
    tmp += ';';
    return tmp;
}





/**
 * A constructor.
 * @param[in] distance Obstacle distance 0-100.
 */
DistancePacket::DistancePacket(const uint8_t &distance) : distance(distance) {}


/**
 * A destructor.
 */
DistancePacket::~DistancePacket() {}


/**
 * A constructor for parsing data.
 * @param[in] data Data string.
 */
DistancePacket::DistancePacket(const std::string &data) {
    if(!data.empty()) {
        int separator = data.find(';');
        std::string parse = data.substr(1, separator - 1);
        this->distance = std::atoi(parse.c_str());
    }
}


/**
 * Get type of packet.
 * @return Type.
 */
char DistancePacket::getType() {
    return 'D';
}


/**
 * Prepare packet for sending.
 * @return String of data.
 */
std::string DistancePacket::prepare() {
    std::string tmp;
    tmp += this->getType();
    tmp += std::to_string(this->distance);
    tmp += Packet::checksum(tmp);
    tmp += ';';
    return tmp;
}





/**
 * A constructor.
 * @param[in] x     Gyroscop data of X axis.
 * @param[in] y     Gyroscop data of Y axis.
 * @param[in] z     Gyroscop data of Z axis.
 */
GyroPacket::GyroPacket(const int8_t &x, const int8_t &y, const int8_t &z) : x(x), y(y), z(z) {}


/**
 * A constructor.
 */
GyroPacket::GyroPacket() {}


/**
 * A destructor.
 */
GyroPacket::~GyroPacket() {}


/**
 * A constructor for parsing data.
 * @param[in] data      Data string.
 */
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


/**
 * Get type of packet.
 * @return Type.
 */
char GyroPacket::getType() {
    return 'G';
}


/**
 * Prepare packet for sending.
 * @return String of data.
 */
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




/**
 * A constructor.
 * @param[in] x     Accelerometer data of X axis.
 * @param[in] y     Accelerometer data of Y axis.
 * @param[in] z     Accelerometer data of Z axis.
 */
AcceloPacket::AcceloPacket(const int8_t &x, const int8_t &y, const int8_t &z) : x(x), y(y), z(z) {}


/**
 * A constructor.
 */
AcceloPacket::AcceloPacket() {}


/**
 * A destructor.
 */
AcceloPacket::~AcceloPacket() {}


/**
 * A constructor for parsing data.
 * @param[in] data      Data string.
 */
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


/**
 * Get type of packet.
 * @return Type.
 */
char AcceloPacket::getType() {
    return 'A';
}


/**
 * Prepare packet for sending.
 * @return String of data.
 */
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





/**
 * A constructor.
 * @param[in] left      Engine speed.
 * @param[in] right     Engine speed.
 */
SpeedPacket::SpeedPacket(const int8_t &left, const int8_t &right) : left(left), right(right) {}


/**
 * A destructor.
 */
SpeedPacket::~SpeedPacket() {}


/**
 * A constructor for parsing data.
 * @param[in] data      Data string.
 */
SpeedPacket::SpeedPacket(const std::string &data) {
    if(!data.empty()) {
        int separator = data.find(' ');
        std::string parse = data.substr(1, separator - 1);
        this->left = std::atoi(parse.c_str());
        parse = data.substr(separator + 1, data.find(';') - separator - 1);
        this->right = std::atoi(parse.c_str());
    }
}


/**
 * Get type of packet.
 * @return Type.
 */
char SpeedPacket::getType() {
    return 'S';
}


/**
 * Prepare packet for sending.
 * @return String of data.
 */
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