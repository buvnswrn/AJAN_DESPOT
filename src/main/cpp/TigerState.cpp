//
// Created by bhuvanesh on 26.05.23.
//

#include <iostream>
#include "TigerState.h"

TigerState::TigerState() {
    someMethod();
}

std::string TigerState::text() const {
    return "Nothing for now";
}
void TigerState::someMethod() {
    std::cout<<"Inside TigerState"<<std::endl;
}