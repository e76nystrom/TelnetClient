//
// Created by Eric Nystrom on 6/23/26.
//

#ifndef SERVER_H
#define SERVER_H

#include "Print.h"

class ServerX: public Print
{
    uint16_t _port = 0;
public:
    void begin(uint16_t port=0)
    { _port = port;}
};

#endif //SERVER_H
