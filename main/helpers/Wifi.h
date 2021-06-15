#ifndef KITE_WIFI_H
#define KITE_WIFI_H

#include <array>

using namespace std;

class Wifi {

    static array<uint8_t , 6> destination;

public:

    static void init(array<uint8_t, 6> destination_mac);
    static void send(uint8_t *data, size_t len);

};

#endif //KITE_WIFI_H