//
// Created by bhuvanesh on 26.05.23.
//

#ifndef POMDP_TIGERSTATE_H
#define POMDP_TIGERSTATE_H
#include <despot/interface/pomdp.h>

class TigerState: public despot::State{
public:
    TigerState();
    static void someMethod();
    std::string text() const;
};


#endif //POMDP_TIGERSTATE_H
